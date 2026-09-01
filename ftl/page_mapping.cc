/*
 * Copyright (C) 2017 CAMELab
 *
 * This file is part of SimpleSSD.
 *
 * SimpleSSD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SimpleSSD is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SimpleSSD.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ftl/page_mapping.hh"

#include <algorithm>
#include <limits>
#include <list>
#include <random>

#include "util/algorithm.hh"
#include "util/bitset.hh"

namespace SimpleSSD {

namespace FTL {

PageMapping::PageMapping(ConfigReader &c, Parameter &p, PAL::PAL *l,
                         DRAM::AbstractDRAM *d)
    : AbstractFTL(p, l, d),
      pPAL(l),
      conf(c),
      lastFreeBlock(param.pageCountToMaxPerf),
      lastFreeBlockIOMap(param.ioUnitInPage),
      bReclaimMore(false) {
  blocks.reserve(param.totalPhysicalBlocks);
  table.reserve(param.totalLogicalBlocks * param.pagesInBlock);

  for (uint32_t i = 0; i < param.totalPhysicalBlocks; i++) {
    freeBlocks.emplace_back(Block(i, param.pagesInBlock, param.ioUnitInPage));
  }

  nFreeBlocks = param.totalPhysicalBlocks;

  status.totalLogicalPages = param.totalLogicalBlocks * param.pagesInBlock;

  // Allocate free blocks
  for (uint32_t i = 0; i < param.pageCountToMaxPerf; i++) {
    lastFreeBlock.at(i) = getFreeBlock(i);
  }

  lastFreeBlockIndex = 0;

  memset(&stat, 0, sizeof(stat));

  bRandomTweak = conf.readBoolean(CONFIG_FTL, FTL_USE_RANDOM_IO_TWEAK);
  bitsetSize = bRandomTweak ? param.ioUnitInPage : 1;

  cmtPolicy = (CMT_POLICY)conf.readInt(CONFIG_FTL, FTL_CMT_POLICY);

  // One CMT entry caches the mapping for a whole superpage, which is
  // bitsetSize sub-page mappings of 8 bytes each (4B LPN + 4B PPN, as in the
  // DFTL paper).  Sizing the cache by bytes therefore has to account for
  // bitsetSize, otherwise a "2 MB" cache silently holds bitsetSize times as
  // much mapping data as its label claims.
  cmtEntryBytes = 8 * bitsetSize;

  float cmtRatio = conf.readFloat(CONFIG_FTL, FTL_CMT_CAPACITY_RATIO);
  if (cmtRatio > 0.0f) {
    cmtCapacity = (uint64_t)((float)status.totalLogicalPages * cmtRatio);
  }
  else {
    uint64_t cmtBytes = conf.readUint(CONFIG_FTL, FTL_CMT_CAPACITY_BYTES);
    cmtCapacity = cmtBytes / cmtEntryBytes;
  }
  if (cmtCapacity < 16) cmtCapacity = 16;
  cmt.reserve(cmtCapacity);
  cmtLFU.reserve(cmtCapacity);

  // DFTL "double read" penalty: NAND flash read latency on CMT miss
  cmtMissLatency = conf.readUint(CONFIG_FTL, FTL_CMT_MISS_LATENCY);

  cmtSpatialPrefetch = conf.readBoolean(CONFIG_FTL, FTL_CMT_SPATIAL_PREFETCH);
  cmtPrefetchWindow = conf.readUint(CONFIG_FTL, FTL_CMT_PREFETCH_WINDOW);
  // NAND flash program latency for dirty write-back on eviction
  cmtWriteBackLatency = conf.readUint(CONFIG_FTL, FTL_CMT_WRITEBACK_LATENCY);
  cmtMinFreq = 0;

  debugprint(LOG_FTL_PAGE_MAPPING,
             "CMT  | Policy %s | %" PRIu64 " entries | %" PRIu64
             " B/entry (bitsetSize %u) | %" PRIu64 " B total",
             cmtPolicy == CMT_POLICY_LFU ? "LFU" : "LRU", cmtCapacity,
             cmtEntryBytes, bitsetSize, cmtCapacity * cmtEntryBytes);
  debugprint(LOG_FTL_PAGE_MAPPING,
             "CMT  | Prefetch %s | window %" PRIu64 " LPNs",
             cmtSpatialPrefetch ? "on" : "off", cmtPrefetchWindow);
}

PageMapping::~PageMapping() {
  debugprint(LOG_FTL_PAGE_MAPPING,
             "CMT  | Prefetch | triggers %" PRIu64 " | insertions %" PRIu64
             " | hits %" PRIu64 " | unused_evict %" PRIu64
             " | resident_prefetched %" PRIu64 " | occupancy %" PRIu64
             " / %" PRIu64,
             stat.cmtPrefetchTriggers, stat.cmtPrefetchInsertions,
             stat.cmtPrefetchHits, stat.cmtPrefetchEvictedUnused,
             countResidentPrefetched(), cmtSize(), cmtCapacity);

  // Flush any remaining dirty CMT entries back to GMT so the mapping
  // table is coherent if inspected after simulation ends.
  flushCMT();
}

void PageMapping::flushCMT() {
  // Write back every dirty CMT entry to the GMT (table) so the on-disk
  // mapping is coherent after the simulation finishes.  Clean entries are
  // already up-to-date in GMT and need no action.
  if (cmtPolicy == CMT_POLICY_LFU) {
    for (auto &entry : cmtLFU) {
      if (entry.second.dirty) {
        table[entry.first] = entry.second.mapping;
      }
    }
  }
  else {
    for (auto &entry : cmt) {
      if (entry.second.first.dirty) {
        table[entry.first] = entry.second.first.mapping;
      }
    }
  }

  cmt.clear();
  cmtOrder.clear();
  cmtLFU.clear();
  cmtFreqBuckets.clear();
  cmtMinFreq = 0;
}

void PageMapping::repairLFUMinFreq() {
  if (cmtFreqBuckets.empty()) {
    cmtMinFreq = 0;
    return;
  }

  cmtMinFreq = cmtFreqBuckets.begin()->first;

  for (const auto &bucket : cmtFreqBuckets) {
    if (bucket.first < cmtMinFreq) {
      cmtMinFreq = bucket.first;
    }
  }
}

void PageMapping::cmtErase(uint64_t lpn) {
  // Drop an LPN whose mapping no longer exists.  No write-back: the caller is
  // destroying the mapping, so pushing it to the GMT would resurrect it.
  if (cmtPolicy == CMT_POLICY_LFU) {
    auto iter = cmtLFU.find(lpn);

    if (iter != cmtLFU.end()) {
      if (iter->second.prefetched) {
        stat.cmtPrefetchEvictedUnused++;
      }
      uint64_t freq = iter->second.freq;
      auto bucket = cmtFreqBuckets.find(freq);

      if (bucket != cmtFreqBuckets.end()) {
        bucket->second.erase(iter->second.listIt);

        if (bucket->second.empty()) {
          cmtFreqBuckets.erase(bucket);

          // Eviction reads cmtFreqBuckets[cmtMinFreq].  If we just emptied
          // that bucket, leave the pointer on a real occupied frequency.
          if (freq == cmtMinFreq) {
            repairLFUMinFreq();
          }
        }
      }

      cmtLFU.erase(iter);
    }
  }
  else {
    auto iter = cmt.find(lpn);

    if (iter != cmt.end()) {
      if (iter->second.first.prefetched) {
        stat.cmtPrefetchEvictedUnused++;
      }
      cmtOrder.erase(iter->second.second);
      cmt.erase(iter);
    }
  }
}

std::vector<std::pair<uint32_t, uint32_t>> *PageMapping::getLiveMapping(
    uint64_t lpn) {
  // Prefer the CMT copy: writes update CMT and mark dirty, so GMT may still
  // hold a sentinel or a previous PPN until the next dirty eviction.
  if (cmtPolicy == CMT_POLICY_LFU) {
    auto it = cmtLFU.find(lpn);

    if (it != cmtLFU.end()) {
      return &it->second.mapping;
    }
  }
  else {
    auto it = cmt.find(lpn);

    if (it != cmt.end()) {
      return &it->second.first.mapping;
    }
  }

  auto gmtIt = table.find(lpn);

  if (gmtIt == table.end()) {
    return nullptr;
  }

  return &gmtIt->second;
}

uint64_t PageMapping::cmtSize() const {
  return cmtPolicy == CMT_POLICY_LFU ? cmtLFU.size() : cmt.size();
}

void PageMapping::resetCMTStats() {
  stat.cmtHits = 0;
  stat.cmtMisses = 0;
  stat.cmtEvictions = 0;
  stat.cmtDirtyEvictions = 0;
  stat.cmtWritebacks = 0;
  stat.cmtGCHits = 0;
  stat.cmtGCMisses = 0;
  stat.cmtPrefetchInsertions = 0;
  stat.cmtPrefetchHits = 0;
  stat.cmtPrefetchEvictedUnused = 0;
  stat.cmtPrefetchTriggers = 0;

  // Clear prefetched flag on all resident entries so warmup prefetches
  // don't leak into measurement stats (which would cause accuracy > 100%).
  for (auto &e : cmt) {
    e.second.first.prefetched = false;
  }
  for (auto &e : cmtLFU) {
    e.second.prefetched = false;
  }
}

bool PageMapping::initialize() {
  uint64_t nPagesToWarmup;
  uint64_t nPagesToInvalidate;
  uint64_t nTotalLogicalPages;
  uint64_t maxPagesBeforeGC;
  uint64_t tick;
  uint64_t valid;
  uint64_t invalid;
  FILLING_MODE mode;

  Request req(param.ioUnitInPage);

  debugprint(LOG_FTL_PAGE_MAPPING, "Initialization started");

  nTotalLogicalPages = param.totalLogicalBlocks * param.pagesInBlock;
  nPagesToWarmup =
      nTotalLogicalPages * conf.readFloat(CONFIG_FTL, FTL_FILL_RATIO);
  nPagesToInvalidate =
      nTotalLogicalPages * conf.readFloat(CONFIG_FTL, FTL_INVALID_PAGE_RATIO);
  mode = (FILLING_MODE)conf.readUint(CONFIG_FTL, FTL_FILLING_MODE);
  maxPagesBeforeGC =
      param.pagesInBlock *
      (param.totalPhysicalBlocks *
           (1 - conf.readFloat(CONFIG_FTL, FTL_GC_THRESHOLD_RATIO)) -
       param.pageCountToMaxPerf);  // # free blocks to maintain

  if (nPagesToWarmup + nPagesToInvalidate > maxPagesBeforeGC) {
    warn("ftl: Too high filling ratio. Adjusting invalidPageRatio.");
    nPagesToInvalidate = maxPagesBeforeGC - nPagesToWarmup;
  }

  debugprint(LOG_FTL_PAGE_MAPPING, "Total logical pages: %" PRIu64,
             nTotalLogicalPages);
  debugprint(LOG_FTL_PAGE_MAPPING,
             "Total logical pages to fill: %" PRIu64 " (%.2f %%)",
             nPagesToWarmup, nPagesToWarmup * 100.f / nTotalLogicalPages);
  debugprint(LOG_FTL_PAGE_MAPPING,
             "Total invalidated pages to create: %" PRIu64 " (%.2f %%)",
             nPagesToInvalidate,
             nPagesToInvalidate * 100.f / nTotalLogicalPages);

  req.ioFlag.set();

  // Step 1. Filling
  if (mode == FILLING_MODE_0 || mode == FILLING_MODE_1) {
    // Sequential
    for (uint64_t i = 0; i < nPagesToWarmup; i++) {
      tick = 0;
      req.lpn = i;
      writeInternal(req, tick, false);
    }
  }
  else {
    // Random
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist(0, nTotalLogicalPages - 1);

    for (uint64_t i = 0; i < nPagesToWarmup; i++) {
      tick = 0;
      req.lpn = dist(gen);
      writeInternal(req, tick, false);
    }
  }

  // Step 2. Invalidating
  if (mode == FILLING_MODE_0) {
    // Sequential
    for (uint64_t i = 0; i < nPagesToInvalidate; i++) {
      tick = 0;
      req.lpn = i;
      writeInternal(req, tick, false);
    }
  }
  else if (mode == FILLING_MODE_1) {
    // Random
    // We can successfully restrict range of LPN to create exact number of
    // invalid pages because we wrote in sequential mannor in step 1.
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist(0, nPagesToWarmup - 1);

    for (uint64_t i = 0; i < nPagesToInvalidate; i++) {
      tick = 0;
      req.lpn = dist(gen);
      writeInternal(req, tick, false);
    }
  }
  else {
    // Random
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist(0, nTotalLogicalPages - 1);

    for (uint64_t i = 0; i < nPagesToInvalidate; i++) {
      tick = 0;
      req.lpn = dist(gen);
      writeInternal(req, tick, false);
    }
  }

  // Report
  calculateTotalPages(valid, invalid);
  debugprint(LOG_FTL_PAGE_MAPPING, "Filling finished. Page status:");
  debugprint(LOG_FTL_PAGE_MAPPING,
             "  Total valid physical pages: %" PRIu64
             " (%.2f %%, target: %" PRIu64 ", error: %" PRId64 ")",
             valid, valid * 100.f / nTotalLogicalPages, nPagesToWarmup,
             (int64_t)(valid - nPagesToWarmup));
  debugprint(LOG_FTL_PAGE_MAPPING,
             "  Total invalid physical pages: %" PRIu64
             " (%.2f %%, target: %" PRIu64 ", error: %" PRId64 ")",
             invalid, invalid * 100.f / nTotalLogicalPages, nPagesToInvalidate,
             (int64_t)(invalid - nPagesToInvalidate));

  // Warm-up drove millions of writes through the CMT, every one of them a
  // compulsory miss.  Leaving those in the counters buries the measured
  // workload's hit rate, so zero the CMT counters here.  Cache *contents*
  // stay warm, which is what a real drive would look like at this point.
  debugprint(LOG_FTL_PAGE_MAPPING,
             "CMT  | Warm-up | %" PRIu64 " hits, %" PRIu64
             " misses discarded | %" PRIu64 " entries resident",
             stat.cmtHits, stat.cmtMisses, cmtSize());
  resetCMTStats();

  debugprint(LOG_FTL_PAGE_MAPPING, "Initialization finished");

  return true;
}

void PageMapping::read(Request &req, uint64_t &tick) {
  uint64_t begin = tick;

  if (req.ioFlag.count() > 0) {
    readInternal(req, tick);

    debugprint(LOG_FTL_PAGE_MAPPING,
               "READ  | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64 " (%" PRIu64
               ")",
               req.lpn, begin, tick, tick - begin);
  }
  else {
    warn("FTL got empty request");
  }

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::READ);
}

void PageMapping::write(Request &req, uint64_t &tick) {
  uint64_t begin = tick;

  if (req.ioFlag.count() > 0) {
    writeInternal(req, tick);

    debugprint(LOG_FTL_PAGE_MAPPING,
               "WRITE | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64 " (%" PRIu64
               ")",
               req.lpn, begin, tick, tick - begin);
  }
  else {
    warn("FTL got empty request");
  }

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::WRITE);
}

void PageMapping::trim(Request &req, uint64_t &tick) {
  uint64_t begin = tick;

  trimInternal(req, tick);

  debugprint(LOG_FTL_PAGE_MAPPING,
             "TRIM  | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64 " (%" PRIu64
             ")",
             req.lpn, begin, tick, tick - begin);

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::TRIM);
}

void PageMapping::format(LPNRange &range, uint64_t &tick) {
  PAL::Request req(param.ioUnitInPage);
  std::vector<uint32_t> list;

  req.ioFlag.set();

  for (auto iter = table.begin(); iter != table.end();) {
    if (iter->first >= range.slpn && iter->first < range.slpn + range.nlp) {
      // Use the live mapping (CMT if dirty/resident, else GMT).  GMT alone
      // can still hold the allocate sentinel or a pre-writeback PPN, which
      // would panic or invalidate the wrong page.
      auto *mappingList = getLiveMapping(iter->first);

      if (mappingList == nullptr) {
        mappingList = &iter->second;
      }

      // Do trim
      for (uint32_t idx = 0; idx < bitsetSize; idx++) {
        auto &mapping = mappingList->at(idx);

        // Skip never-mapped sub-pages (sentinel block index).
        if (mapping.first >= param.totalPhysicalBlocks) {
          continue;
        }

        auto block = blocks.find(mapping.first);

        if (block == blocks.end()) {
          panic("Block is not in use");
        }

        block->second.invalidate(mapping.second, idx);

        // Collect block indices
        list.push_back(mapping.first);
      }

      // Drop the cache entry without write-back — the mapping is being
      // destroyed, so pushing it back to GMT would resurrect it.
      cmtErase(iter->first);

      iter = table.erase(iter);
    }
    else {
      iter++;
    }
  }

  // Get blocks to erase
  std::sort(list.begin(), list.end());
  auto last = std::unique(list.begin(), list.end());
  list.erase(last, list.end());

  // Do GC only in specified blocks
  doGarbageCollection(list, tick);

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::FORMAT);
}

Status *PageMapping::getStatus(uint64_t lpnBegin, uint64_t lpnEnd) {
  status.freePhysicalBlocks = nFreeBlocks;

  if (lpnBegin == 0 && lpnEnd >= status.totalLogicalPages) {
    status.mappedLogicalPages = table.size();
  }
  else {
    status.mappedLogicalPages = 0;

    for (uint64_t lpn = lpnBegin; lpn < lpnEnd; lpn++) {
      if (table.count(lpn) > 0) {
        status.mappedLogicalPages++;
      }
    }
  }

  return &status;
}

float PageMapping::freeBlockRatio() {
  return (float)nFreeBlocks / param.totalPhysicalBlocks;
}

uint32_t PageMapping::convertBlockIdx(uint32_t blockIdx) {
  return blockIdx % param.pageCountToMaxPerf;
}

uint32_t PageMapping::getFreeBlock(uint32_t idx) {
  uint32_t blockIndex = 0;

  if (idx >= param.pageCountToMaxPerf) {
    panic("Index out of range");
  }

  if (nFreeBlocks > 0) {
    // Search block which is blockIdx % param.pageCountToMaxPerf == idx
    auto iter = freeBlocks.begin();

    for (; iter != freeBlocks.end(); iter++) {
      blockIndex = iter->getBlockIndex();

      if (blockIndex % param.pageCountToMaxPerf == idx) {
        break;
      }
    }

    // Sanity check
    if (iter == freeBlocks.end()) {
      // Just use first one
      iter = freeBlocks.begin();
      blockIndex = iter->getBlockIndex();
    }

    // Insert found block to block list
    if (blocks.find(blockIndex) != blocks.end()) {
      panic("Corrupted");
    }

    blocks.emplace(blockIndex, std::move(*iter));

    // Remove found block from free block list
    freeBlocks.erase(iter);
    nFreeBlocks--;
  }
  else {
    panic("No free block left");
  }

  return blockIndex;
}

uint32_t PageMapping::getLastFreeBlock(Bitset &iomap) {
  if (!bRandomTweak || (lastFreeBlockIOMap & iomap).any()) {
    // Update lastFreeBlockIndex
    lastFreeBlockIndex++;

    if (lastFreeBlockIndex == param.pageCountToMaxPerf) {
      lastFreeBlockIndex = 0;
    }

    lastFreeBlockIOMap = iomap;
  }
  else {
    lastFreeBlockIOMap |= iomap;
  }

  auto freeBlock = blocks.find(lastFreeBlock.at(lastFreeBlockIndex));

  // Sanity check
  if (freeBlock == blocks.end()) {
    panic("Corrupted");
  }

  // If current free block is full, get next block
  if (freeBlock->second.getNextWritePageIndex() == param.pagesInBlock) {
    lastFreeBlock.at(lastFreeBlockIndex) = getFreeBlock(lastFreeBlockIndex);

    bReclaimMore = true;
  }

  return lastFreeBlock.at(lastFreeBlockIndex);
}

// calculate weight of each block regarding victim selection policy
void PageMapping::calculateVictimWeight(
    std::vector<std::pair<uint32_t, float>> &weight, const EVICT_POLICY policy,
    uint64_t tick) {
  float temp;

  weight.reserve(blocks.size());

  switch (policy) {
    case POLICY_GREEDY:
    case POLICY_RANDOM:
    case POLICY_DCHOICE:
      for (auto &iter : blocks) {
        if (iter.second.getNextWritePageIndex() != param.pagesInBlock) {
          continue;
        }

        weight.push_back({iter.first, iter.second.getValidPageCountRaw()});
      }

      break;
    case POLICY_COST_BENEFIT:
      for (auto &iter : blocks) {
        if (iter.second.getNextWritePageIndex() != param.pagesInBlock) {
          continue;
        }

        temp = (float)(iter.second.getValidPageCountRaw()) / param.pagesInBlock;

        weight.push_back(
            {iter.first,
             temp / ((1 - temp) * (tick - iter.second.getLastAccessedTime()))});
      }

      break;
    default:
      panic("Invalid evict policy");
  }
}

void PageMapping::selectVictimBlock(std::vector<uint32_t> &list,
                                    uint64_t &tick) {
  static const GC_MODE mode = (GC_MODE)conf.readInt(CONFIG_FTL, FTL_GC_MODE);
  static const EVICT_POLICY policy =
      (EVICT_POLICY)conf.readInt(CONFIG_FTL, FTL_GC_EVICT_POLICY);
  static uint32_t dChoiceParam =
      conf.readUint(CONFIG_FTL, FTL_GC_D_CHOICE_PARAM);
  uint64_t nBlocks = conf.readUint(CONFIG_FTL, FTL_GC_RECLAIM_BLOCK);
  std::vector<std::pair<uint32_t, float>> weight;

  list.clear();

  // Calculate number of blocks to reclaim
  if (mode == GC_MODE_0) {
    // DO NOTHING
  }
  else if (mode == GC_MODE_1) {
    static const float t = conf.readFloat(CONFIG_FTL, FTL_GC_RECLAIM_THRESHOLD);

    nBlocks = param.totalPhysicalBlocks * t - nFreeBlocks;
  }
  else {
    panic("Invalid GC mode");
  }

  // reclaim one more if last free block fully used
  if (bReclaimMore) {
    nBlocks += param.pageCountToMaxPerf;

    bReclaimMore = false;
  }

  // Calculate weights of all blocks
  calculateVictimWeight(weight, policy, tick);

  if (policy == POLICY_RANDOM || policy == POLICY_DCHOICE) {
    uint64_t randomRange =
        policy == POLICY_RANDOM ? nBlocks : dChoiceParam * nBlocks;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint64_t> dist(0, weight.size() - 1);
    std::vector<std::pair<uint32_t, float>> selected;

    while (selected.size() < randomRange) {
      uint64_t idx = dist(gen);

      if (weight.at(idx).first < std::numeric_limits<uint32_t>::max()) {
        selected.push_back(weight.at(idx));
        weight.at(idx).first = std::numeric_limits<uint32_t>::max();
      }
    }

    weight = std::move(selected);
  }

  // Sort weights
  std::sort(
      weight.begin(), weight.end(),
      [](std::pair<uint32_t, float> a, std::pair<uint32_t, float> b) -> bool {
        return a.second < b.second;
      });

  // Select victims from the blocks with the lowest weight
  nBlocks = MIN(nBlocks, weight.size());

  for (uint64_t i = 0; i < nBlocks; i++) {
    list.push_back(weight.at(i).first);
  }

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::SELECT_VICTIM_BLOCK);
}

void PageMapping::doGarbageCollection(std::vector<uint32_t> &blocksToReclaim,
                                      uint64_t &tick) {
  PAL::Request req(param.ioUnitInPage);
  std::vector<PAL::Request> readRequests;
  std::vector<PAL::Request> writeRequests;
  std::vector<PAL::Request> eraseRequests;
  std::vector<uint64_t> lpns;
  Bitset bit(param.ioUnitInPage);
  uint64_t beginAt;
  uint64_t readFinishedAt = tick;
  uint64_t writeFinishedAt = tick;
  uint64_t eraseFinishedAt = tick;

  if (blocksToReclaim.size() == 0) {
    return;
  }

  // For all blocks to reclaim, collecting request structure only
  for (auto &iter : blocksToReclaim) {
    auto block = blocks.find(iter);

    if (block == blocks.end()) {
      panic("Invalid block");
    }

    // Copy valid pages to free block
    for (uint32_t pageIndex = 0; pageIndex < param.pagesInBlock; pageIndex++) {
      // Valid?
      if (block->second.getPageInfo(pageIndex, lpns, bit)) {
        if (!bRandomTweak) {
          bit.set();
        }

        // Retrive free block
        auto freeBlock = blocks.find(getLastFreeBlock(bit));

        // Issue Read
        req.blockIndex = block->first;
        req.pageIndex = pageIndex;
        req.ioFlag = bit;

        readRequests.push_back(req);

        // Update mapping table
        uint32_t newBlockIdx = freeBlock->first;

        for (uint32_t idx = 0; idx < bitsetSize; idx++) {
          if (bit.test(idx)) {
            // Invalidate
            block->second.invalidate(pageIndex, idx);

            // GC also updates mappings — go through CMT for consistency
            auto &gcMappingData = *accessCMT(lpns.at(idx), true, tick, true);
            auto &mapping = gcMappingData.at(idx);

            uint32_t newPageIdx = freeBlock->second.getNextWritePageIndex(idx);

            mapping.first = newBlockIdx;
            mapping.second = newPageIdx;

            freeBlock->second.write(newPageIdx, lpns.at(idx), idx, beginAt);

            // Issue Write
            req.blockIndex = newBlockIdx;
            req.pageIndex = newPageIdx;

            if (bRandomTweak) {
              req.ioFlag.reset();
              req.ioFlag.set(idx);
            }
            else {
              req.ioFlag.set();
            }

            writeRequests.push_back(req);

            stat.validPageCopies++;
          }
        }

        stat.validSuperPageCopies++;
      }
    }

    // Erase block
    req.blockIndex = block->first;
    req.pageIndex = 0;
    req.ioFlag.set();

    eraseRequests.push_back(req);
  }

  // Do actual I/O here
  // This handles PAL2 limitation (SIGSEGV, infinite loop, or so-on)
  for (auto &iter : readRequests) {
    beginAt = tick;

    pPAL->read(iter, beginAt);

    readFinishedAt = MAX(readFinishedAt, beginAt);
  }

  for (auto &iter : writeRequests) {
    beginAt = readFinishedAt;

    pPAL->write(iter, beginAt);

    writeFinishedAt = MAX(writeFinishedAt, beginAt);
  }

  for (auto &iter : eraseRequests) {
    beginAt = readFinishedAt;

    eraseInternal(iter, beginAt);

    eraseFinishedAt = MAX(eraseFinishedAt, beginAt);
  }

    tick = MAX(writeFinishedAt, eraseFinishedAt);
  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::DO_GARBAGE_COLLECTION);
}

std::vector<std::pair<uint32_t, uint32_t>> *
PageMapping::accessCMT(uint64_t lpn, bool isWrite, uint64_t &tick, bool isGC,
                       bool allocate) {
  if (cmtPolicy == CMT_POLICY_LFU) {
    return accessCMT_LFU(lpn, isWrite, tick, isGC, allocate);
  }

  return accessCMT_LRU(lpn, isWrite, tick, isGC, allocate);
}

bool PageMapping::cmtContains(uint64_t lpn) const {
  return cmtPolicy == CMT_POLICY_LFU ? cmtLFU.count(lpn) != 0
                                     : cmt.count(lpn) != 0;
}

std::vector<uint64_t> PageMapping::collectPrefetchCandidates(
    uint64_t lpn) const {
  std::vector<uint64_t> candidates;
  if (cmtPrefetchWindow == 0) {
    return candidates;
  }

  uint64_t groupStart = (lpn / cmtPrefetchWindow) * cmtPrefetchWindow;
  uint64_t groupEnd =
      std::min(groupStart + cmtPrefetchWindow, status.totalLogicalPages);

  for (uint64_t candidateLpn = groupStart; candidateLpn < groupEnd;
       ++candidateLpn) {
    if (candidateLpn == lpn) continue;
    if (cmtContains(candidateLpn)) continue;
    if (table.find(candidateLpn) == table.end()) continue;
    candidates.push_back(candidateLpn);
  }

  uint64_t maxBatch = cmtCapacity > 1 ? cmtCapacity - 1 : 0;
  if (candidates.size() > maxBatch) {
    candidates.resize(maxBatch);
  }

  return candidates;
}

void PageMapping::evictForPrefetchBatch(size_t batchSize, uint64_t &tick) {
  // Prefetch-induced victims still write GMT so mappings stay coherent, but
  // charging CMTWriteBackLatency per entry would serialize hundreds of NAND
  // programs on one miss.  Charge at most one translation-page program if any
  // dirty victim was displaced.
  uint64_t dirtyBefore = stat.cmtDirtyEvictions;

  if (cmtPolicy == CMT_POLICY_LFU) {
    while (cmtLFU.size() + batchSize + 1 > cmtCapacity && !cmtLFU.empty()) {
      evictOneLFUVictim(tick, false);
    }
  }
  else {
    while (cmt.size() + batchSize + 1 > cmtCapacity && !cmt.empty()) {
      evictOneLRUVictim(tick, false);
    }
  }

  if (stat.cmtDirtyEvictions > dirtyBefore) {
    tick += cmtWriteBackLatency;
  }
}

uint64_t PageMapping::insertPrefetchBatchLRU(
    const std::vector<uint64_t> &candidates) {
  cmt.reserve(cmt.size() + candidates.size());
  uint64_t inserted = 0;

  for (uint64_t candidateLpn : candidates) {
    if (cmt.size() >= cmtCapacity) break;
    auto candGmtIt = table.find(candidateLpn);
    if (candGmtIt == table.end()) continue;

    cmtOrder.push_back(candidateLpn);
    auto result = cmt.emplace(
        candidateLpn,
        std::make_pair(CMTEntry{candGmtIt->second, false, true},
                       std::prev(cmtOrder.end())));
    if (!result.second) {
      cmtOrder.pop_back();
      continue;
    }
    inserted++;
  }

  stat.cmtPrefetchInsertions += inserted;
  return inserted;
}

uint64_t PageMapping::insertPrefetchBatchLFU(
    const std::vector<uint64_t> &candidates) {
  cmtLFU.reserve(cmtLFU.size() + candidates.size());
  uint64_t inserted = 0;

  for (uint64_t candidateLpn : candidates) {
    if (cmtLFU.size() >= cmtCapacity) break;
    auto candGmtIt = table.find(candidateLpn);
    if (candGmtIt == table.end()) continue;

    cmtMinFreq = 1;
    cmtFreqBuckets[1].push_back(candidateLpn);

    CMTEntryLFU candEntry;
    candEntry.mapping = candGmtIt->second;
    candEntry.dirty = false;
    candEntry.freq = 1;
    candEntry.listIt = std::prev(cmtFreqBuckets[1].end());
    candEntry.prefetched = true;

    auto result = cmtLFU.emplace(candidateLpn, std::move(candEntry));
    if (!result.second) {
      cmtFreqBuckets[1].pop_back();
      if (cmtFreqBuckets[1].empty()) {
        cmtFreqBuckets.erase(1);
        repairLFUMinFreq();
      }
      continue;
    }
    inserted++;
  }

  stat.cmtPrefetchInsertions += inserted;
  return inserted;
}

void PageMapping::chargePrefetchDRAM(uint64_t nEntries, uint64_t &tick) {
  if (nEntries == 0 || pDRAM == nullptr) {
    return;
  }
  pDRAM->read(nullptr, nEntries * cmtEntryBytes, tick);
}

std::vector<std::pair<uint32_t, uint32_t>> *PageMapping::cmtMappingOf(
    uint64_t lpn) {
  if (cmtPolicy == CMT_POLICY_LFU) {
    auto it = cmtLFU.find(lpn);
    if (it == cmtLFU.end()) {
      panic("CMT-LFU: demand LPN missing after insert");
    }
    return &it->second.mapping;
  }

  auto it = cmt.find(lpn);
  if (it == cmt.end()) {
    panic("CMT: demand LPN missing after insert");
  }
  return &it->second.first.mapping;
}

uint64_t PageMapping::countResidentPrefetched() const {
  uint64_t n = 0;
  if (cmtPolicy == CMT_POLICY_LFU) {
    for (const auto &e : cmtLFU) {
      if (e.second.prefetched) n++;
    }
  }
  else {
    for (const auto &e : cmt) {
      if (e.second.first.prefetched) n++;
    }
  }
  return n;
}

// ════════════════════════════════════════════════════════════════════════════
// CMT POLICY: LRU — evict the entry that was accessed least recently.
//
// cmtOrder holds every cached LPN with the most recently used at the front.
// cmt maps LPN → {entry, iterator into cmtOrder}, so promoting on a hit is an
// O(1) list splice and picking a victim is an O(1) read of cmtOrder.back().
// ════════════════════════════════════════════════════════════════════════════
void PageMapping::evictOneLRUVictim(uint64_t &tick, bool chargeWriteBack) {
  if (cmtOrder.empty()) return;
  uint64_t evictLpn = cmtOrder.back();
  auto evictIt = cmt.find(evictLpn);
  if (evictIt != cmt.end()) {
    if (evictIt->second.first.prefetched) {
      stat.cmtPrefetchEvictedUnused++;
    }
    cmtOrder.pop_back();
    stat.cmtEvictions++;

    if (evictIt->second.first.dirty) {
      table[evictLpn] = evictIt->second.first.mapping;
      stat.cmtDirtyEvictions++;
      stat.cmtWritebacks++;
      if (chargeWriteBack) {
        tick += cmtWriteBackLatency;
      }
    }
    cmt.erase(evictIt);
  }
}

std::vector<std::pair<uint32_t, uint32_t>> *
PageMapping::accessCMT_LRU(uint64_t lpn, bool isWrite, uint64_t &tick,
                           bool isGC, bool allocate) {
  auto it = cmt.find(lpn);

  // ────────────────────────────────────────────────
  // CACHE HIT — LPN is already in the CMT
  // ────────────────────────────────────────────────
  if (it != cmt.end()) {
    // Separate GC hits from user hits
    if (isGC) {
      stat.cmtGCHits++;
    } else {
      stat.cmtHits++;
    }

    // Move to front of LRU list (most recently used)
    cmtOrder.splice(cmtOrder.begin(), cmtOrder, it->second.second);

    // Mark dirty if this is a write — needs write-back on eviction
    if (isWrite) {
      it->second.first.dirty = true;
    }

    if (it->second.first.prefetched && !isGC) {
      it->second.first.prefetched = false;
      stat.cmtPrefetchHits++;
    }

    return &it->second.first.mapping;
  }

  // ────────────────────────────────────────────────
  // CACHE MISS — LPN is not in CMT
  // ────────────────────────────────────────────────
  // Separate GC misses from user misses
  if (isGC) {
    stat.cmtGCMisses++;
  } else {
    stat.cmtMisses++;
  }

  auto gmtIt = table.find(lpn);

  // Lookup-only caller (read/trim) and the LPN was never written.  There is
  // nothing to cache, so do not manufacture a mapping: doing so would grow the
  // GMT without bound and fill the CMT with entries that can never hit.
  if (gmtIt == table.end() && !allocate) {
    return nullptr;
  }

  // Edge Case 1: CMT is full — evict the LRU entry (back of list)
  if (cmt.size() >= cmtCapacity) {
    evictOneLRUVictim(tick);

    // A dirty write-back can insert into `table` and rehash it, which
    // invalidates every iterator including gmtIt.  Re-find before use.
    gmtIt = table.find(lpn);
  }

  bool paidMissLatency = false;
  // Edge Case 3: Brand-new LPN — never written before, not in GMT either
  // This happens on the very first write to a logical page
  if (gmtIt == table.end()) {
    auto ret = table.emplace(
        lpn,
        std::vector<std::pair<uint32_t, uint32_t>>(
            bitsetSize, {param.totalPhysicalBlocks, param.pagesInBlock}));

    if (!ret.second) {
      panic("CMT: Failed to create new GMT entry for LPN");
    }

    gmtIt = ret.first;
    // No flash read penalty for brand-new pages — nothing to load from NAND
  }
  else {
    // Edge Case 4: Existing LPN fetched from GMT
    // DFTL "double read" — reading the translation page from NAND flash
    tick += cmtMissLatency;
    paidMissLatency = true;
  }

  // ── Spatial prefetch (LRU version) ──────────────────────────────────────
  //
  // ORDER IS CRITICAL:
  //   Phase 1: collect candidates
  //   Phase 2: evict room for batch + 1 slot reserved for the demand entry
  //   Phase 3: insert the demand entry (safe — Phase 2 cannot evict it)
  //   Phase 4: fill prefetch batch at LRU end (lowest eviction priority)
  //
  // The demand entry is inserted AFTER Phase 2 so it can never be chosen
  // as a victim by the batch-evict loop.  The "+1" in Phase 2's condition
  // holds one slot in reserve for the demand entry itself.
  // ────────────────────────────────────────────────────────────────────────

  if (cmtSpatialPrefetch && !isGC && paidMissLatency) {
    std::vector<uint64_t> candidates = collectPrefetchCandidates(lpn);

    if (!candidates.empty()) {
      stat.cmtPrefetchTriggers++;
      evictForPrefetchBatch(candidates.size(), tick);
      gmtIt = table.find(lpn);
      if (gmtIt == table.end()) {
        panic("CMT: demand LPN missing from GMT after prefetch eviction");
      }
    }

    // Reserve before demand + batch insert so later emplace cannot rehash
    // and invalidate the mapping pointer returned to read/writeInternal.
    cmt.reserve(cmt.size() + candidates.size() + 1);

    cmtOrder.push_front(lpn);
    cmt.emplace(lpn, std::make_pair(CMTEntry{gmtIt->second, isWrite, false},
                                    cmtOrder.begin()));

    uint64_t nPref = insertPrefetchBatchLRU(candidates);
    chargePrefetchDRAM(nPref, tick);

    if (cmt.size() > cmtCapacity) {
      panic("CMT: occupancy exceeded capacity after prefetch");
    }

    return cmtMappingOf(lpn);
  }

  // Prefetch disabled (or GC path, or brand-new LPN): plain insert at MRU.
  cmtOrder.push_front(lpn);
  auto insertResult = cmt.emplace(
      lpn,
      std::make_pair(CMTEntry{gmtIt->second, isWrite, false},
                     cmtOrder.begin()));

  return &insertResult.first->second.first.mapping;
}

// ════════════════════════════════════════════════════════════════════════════
// CMT POLICY: LFU — evict the entry with the fewest lifetime accesses.
//
// HOW LFU DIFFERS FROM LRU:
//   LRU evicts the entry that was accessed LEAST RECENTLY.
//   LFU evicts the entry that has been accessed the FEWEST TIMES overall.
//
// WHY LFU CAN BE BETTER:
//   Hot pages (e.g. a frequently-read metadata page) stay in cache
//   even if they weren't accessed in the last few operations.
//   Under LRU, a sequential scan can flush the cache of hot pages.
//   LFU is immune to this "cache pollution" from one-time accesses.
//
// WHY LFU CAN BE WORSE:
//   Historically hot pages that are no longer needed stay in cache
//   a long time because their count is high ("cache poisoning").
//   LRU adapts to changing workload patterns faster.
//
// TIE-BREAKING: When two LPNs have the same frequency, evict the one
//   that was accessed least recently (LRU-within-LFU). This is the
//   standard approach and avoids arbitrary eviction ordering.
//
// COMPLEXITY: O(1) hit, O(1) eviction — same as LRU.
//   (via the frequency-bucket algorithm by Shah, Mitra, Matani 2010)
//
// Select this policy with `CMTPolicy = 1` in the FTL config section.
// ════════════════════════════════════════════════════════════════════════════
void PageMapping::evictOneLFUVictim(uint64_t &tick, bool chargeWriteBack) {
  auto minBucket = cmtFreqBuckets.find(cmtMinFreq);
  if (minBucket == cmtFreqBuckets.end() || minBucket->second.empty()) {
    repairLFUMinFreq();
    minBucket = cmtFreqBuckets.find(cmtMinFreq);
  }
  if (minBucket == cmtFreqBuckets.end() || minBucket->second.empty()) {
    panic("CMT-LFU: cache full but no eviction candidate");
  }

  uint64_t evictLpn = minBucket->second.back();
  minBucket->second.pop_back();

  if (minBucket->second.empty()) {
    cmtFreqBuckets.erase(minBucket);
  }

  stat.cmtEvictions++;

  auto evictIt = cmtLFU.find(evictLpn);
  if (evictIt != cmtLFU.end()) {
    if (evictIt->second.prefetched) {
      stat.cmtPrefetchEvictedUnused++;
    }
    if (evictIt->second.dirty) {
      table[evictLpn] = evictIt->second.mapping;
      stat.cmtDirtyEvictions++;
      stat.cmtWritebacks++;
      if (chargeWriteBack) {
        tick += cmtWriteBackLatency;
      }
    }
    cmtLFU.erase(evictIt);
  } else {
    panic("CMT-LFU: frequency bucket and map are out of sync");
  }
}

std::vector<std::pair<uint32_t, uint32_t>> *
PageMapping::accessCMT_LFU(uint64_t lpn, bool isWrite, uint64_t &tick,
                           bool isGC, bool allocate) {
  auto it = cmtLFU.find(lpn);

  // ──────────────────────────────────────────────────────────
  // CACHE HIT — LPN is in the LFU store
  // ──────────────────────────────────────────────────────────
  if (it != cmtLFU.end()) {
    // Count the hit (same as LRU version)
    if (isGC) { stat.cmtGCHits++; } else { stat.cmtHits++; }

    CMTEntryLFU &entry = it->second;

    // ── Promote: move LPN from bucket[f] → bucket[f+1] ──────
    //
    // Step 1: Remove from old bucket
    uint64_t oldFreq = entry.freq;
    cmtFreqBuckets[oldFreq].erase(entry.listIt);
    //   If the old bucket is now empty AND it was the minimum,
    //   the minimum must rise by 1 (the only LPN at min moved up).
    if (cmtFreqBuckets[oldFreq].empty()) {
      cmtFreqBuckets.erase(oldFreq);   // clean up empty bucket
      if (cmtMinFreq == oldFreq) {
        cmtMinFreq = oldFreq + 1;      // min can only go up by 1 on a hit
      }
    }

    // Step 2: Insert into new bucket at the FRONT (= most recently used)
    //   so that within the same frequency, tie-breaking is by recency.
    uint64_t newFreq = oldFreq + 1;
    entry.freq = newFreq;
    cmtFreqBuckets[newFreq].push_front(lpn);
    entry.listIt = cmtFreqBuckets[newFreq].begin();
    // ────────────────────────────────────────────────────────

    // Mark dirty on write (same as LRU)
    if (isWrite) { entry.dirty = true; }

    if (entry.prefetched && !isGC) {
      entry.prefetched = false;
      stat.cmtPrefetchHits++;
    }

    return &entry.mapping;
  }

  // ──────────────────────────────────────────────────────────
  // CACHE MISS — LPN is not in the LFU store
  // ──────────────────────────────────────────────────────────
  if (isGC) { stat.cmtGCMisses++; } else { stat.cmtMisses++; }

  auto gmtIt = table.find(lpn);

  // Lookup-only caller (read/trim) and the LPN was never written — nothing to
  // cache.  See the LRU version for why manufacturing a mapping here is wrong.
  if (gmtIt == table.end() && !allocate) {
    return nullptr;
  }

  // ── Eviction: remove the least-frequently-used entry ─────
  //   The eviction candidate is always at:
  //     cmtFreqBuckets[cmtMinFreq].back()
  //   because:
  //     - cmtMinFreq tracks the globally smallest occupied bucket.
  //     - .back() is the LRU entry within that bucket (tie-break).
  //
  if (cmtLFU.size() >= cmtCapacity) {
    evictOneLFUVictim(tick);

    // A dirty write-back can insert into `table` and rehash it, which
    // invalidates every iterator including gmtIt.  Re-find before use.
    gmtIt = table.find(lpn);
  }
  // ─────────────────────────────────────────────────────────

  // Load from GMT (identical to LRU version)
  bool paidMissLatency = false;
  if (gmtIt == table.end()) {
    // Brand-new LPN — first write ever, not in GMT
    auto ret = table.emplace(
        lpn,
        std::vector<std::pair<uint32_t, uint32_t>>(
            bitsetSize, {param.totalPhysicalBlocks, param.pagesInBlock}));
    if (!ret.second) { panic("CMT-LFU: Failed to create GMT entry"); }
    gmtIt = ret.first;
    // No flash read penalty — nothing to load
  } else {
    // Existing LPN — pay the NAND translation-page read cost
    tick += cmtMissLatency;
    paidMissLatency = true;
  }

  // ── Spatial prefetch (LFU version) ──────────────────────────────────────
  //
  // ORDER IS CRITICAL — same guarantee as the LRU path:
  //   Phase 1: collect candidates
  //   Phase 2: evict room for batch + 1 slot reserved for the demand entry
  //   Phase 3: insert the demand entry (after eviction — cannot be a victim)
  //   Phase 4: fill prefetch batch at freq=1 (lowest eviction priority)
  //
  // The demand entry is inserted in Phase 3, AFTER Phase 2's eviction loop,
  // so Phase 2 can never choose it as a victim.  The "+1" in Phase 2's
  // condition holds exactly one slot in reserve for the demand entry.
  // ────────────────────────────────────────────────────────────────────────

  if (cmtSpatialPrefetch && !isGC && paidMissLatency) {
    std::vector<uint64_t> candidates = collectPrefetchCandidates(lpn);

    if (!candidates.empty()) {
      stat.cmtPrefetchTriggers++;
      evictForPrefetchBatch(candidates.size(), tick);
      gmtIt = table.find(lpn);
      if (gmtIt == table.end()) {
        panic("CMT-LFU: demand LPN missing from GMT after prefetch eviction");
      }
    }

    cmtLFU.reserve(cmtLFU.size() + candidates.size() + 1);

    cmtMinFreq = 1;
    cmtFreqBuckets[1].push_front(lpn);

    CMTEntryLFU newEntry;
    newEntry.mapping = gmtIt->second;
    newEntry.dirty = isWrite;
    newEntry.freq = 1;
    newEntry.listIt = cmtFreqBuckets[1].begin();
    newEntry.prefetched = false;

    cmtLFU.emplace(lpn, std::move(newEntry));

    uint64_t nPref = insertPrefetchBatchLFU(candidates);
    chargePrefetchDRAM(nPref, tick);

    if (cmtLFU.size() > cmtCapacity) {
      panic("CMT-LFU: occupancy exceeded capacity after prefetch");
    }

    return cmtMappingOf(lpn);
  }

  // Prefetch disabled (or GC path, or brand-new LPN): plain insert at freq=1.
  // ── Insertion: new entry always starts at frequency = 1 ──────────────────
  cmtMinFreq = 1;
  cmtFreqBuckets[1].push_front(lpn);

  CMTEntryLFU newEntry;
  newEntry.mapping    = gmtIt->second;
  newEntry.dirty      = isWrite;
  newEntry.freq       = 1;
  newEntry.listIt     = cmtFreqBuckets[1].begin();
  newEntry.prefetched = false;

  auto insertResult = cmtLFU.emplace(lpn, std::move(newEntry));
  // ──────────────────────────────────────────────────────────────────────────

  return &insertResult.first->second.mapping;
}

void PageMapping::readInternal(Request &req, uint64_t &tick) {
  PAL::Request palRequest(req);
  uint64_t beginAt;
  uint64_t finishedAt = tick;

  // ── CMT lookup (replaces direct table.find) ──────────────
  // allocate=false: a read of a never-written LPN must not create a mapping.
  auto *mappingData = accessCMT(req.lpn, false, tick, false, false);

  // Check if there is actually a valid mapping (non-empty)
  bool hasValidMapping = false;
  if (mappingData != nullptr) {
    for (uint32_t idx = 0; idx < bitsetSize; idx++) {
      if (mappingData->at(idx).first < param.totalPhysicalBlocks) {
        hasValidMapping = true;
        break;
      }
    }
  }

  if (hasValidMapping) {
    if (bRandomTweak) {
      pDRAM->read(mappingData, 8 * req.ioFlag.count(), tick);
    }
    else {
      pDRAM->read(mappingData, 8, tick);
    }

    for (uint32_t idx = 0; idx < bitsetSize; idx++) {
      if (req.ioFlag.test(idx) || !bRandomTweak) {
        auto &mapping = mappingData->at(idx);

        if (mapping.first < param.totalPhysicalBlocks &&
            mapping.second < param.pagesInBlock) {
          palRequest.blockIndex = mapping.first;
          palRequest.pageIndex = mapping.second;

          if (bRandomTweak) {
            palRequest.ioFlag.reset();
            palRequest.ioFlag.set(idx);
          }
          else {
            palRequest.ioFlag.set();
          }

          auto block = blocks.find(palRequest.blockIndex);

          if (block == blocks.end()) {
            panic("Block is not in use");
          }

          beginAt = tick;

          block->second.read(palRequest.pageIndex, idx, beginAt);
          pPAL->read(palRequest, beginAt);

          finishedAt = MAX(finishedAt, beginAt);
        }
      }
    }

    tick = finishedAt;
    tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::READ_INTERNAL);
  }
}

void PageMapping::writeInternal(Request &req, uint64_t &tick, bool sendToPAL) {
  PAL::Request palRequest(req);
  std::unordered_map<uint32_t, Block>::iterator block;
  uint64_t beginAt;
  uint64_t finishedAt = tick;
  bool readBeforeWrite = false;

  // ── CMT lookup — isWrite=true marks entry dirty immediately ──
  // A write does create a mapping, so allocate=true (the default).
  auto &mappingData = *accessCMT(req.lpn, true, tick);

  // Check if a previous mapping already exists
  bool hadPreviousMapping = false;
  for (uint32_t idx = 0; idx < bitsetSize; idx++) {
    if (mappingData.at(idx).first < param.totalPhysicalBlocks) {
      hadPreviousMapping = true;
      break;
    }
  }

  if (hadPreviousMapping) {
    for (uint32_t idx = 0; idx < bitsetSize; idx++) {
      if (req.ioFlag.test(idx) || !bRandomTweak) {
        auto &mapping = mappingData.at(idx);

        if (mapping.first < param.totalPhysicalBlocks &&
            mapping.second < param.pagesInBlock) {
          block = blocks.find(mapping.first);

          // Invalidate current page
          block->second.invalidate(mapping.second, idx);
        }
      }
    }
  }

  // Write data to free block
  block = blocks.find(getLastFreeBlock(req.ioFlag));

  if (block == blocks.end()) {
    panic("No such block");
  }

  if (sendToPAL) {
    if (bRandomTweak) {
      pDRAM->read(&mappingData, 8 * req.ioFlag.count(), tick);
      pDRAM->write(&mappingData, 8 * req.ioFlag.count(), tick);
    }
    else {
      pDRAM->read(&mappingData, 8, tick);
      pDRAM->write(&mappingData, 8, tick);
    }
  }

  if (!bRandomTweak && !req.ioFlag.all()) {
    // We have to read old data
    readBeforeWrite = true;
  }

  for (uint32_t idx = 0; idx < bitsetSize; idx++) {
    if (req.ioFlag.test(idx) || !bRandomTweak) {
      uint32_t pageIndex = block->second.getNextWritePageIndex(idx);
      auto &mapping = mappingData.at(idx);

      beginAt = tick;

      block->second.write(pageIndex, req.lpn, idx, beginAt);

      // Read old data if needed (Only executed when bRandomTweak = false)
      // Maybe some other init procedures want to perform 'partial-write'
      // So check sendToPAL variable
      if (readBeforeWrite && sendToPAL) {
        palRequest.blockIndex = mapping.first;
        palRequest.pageIndex = mapping.second;

        // We don't need to read old data
        palRequest.ioFlag = req.ioFlag;
        palRequest.ioFlag.flip();

        pPAL->read(palRequest, beginAt);
      }

      // update mapping to table
      mapping.first = block->first;
      mapping.second = pageIndex;

      if (sendToPAL) {
        palRequest.blockIndex = block->first;
        palRequest.pageIndex = pageIndex;

        if (bRandomTweak) {
          palRequest.ioFlag.reset();
          palRequest.ioFlag.set(idx);
        }
        else {
          palRequest.ioFlag.set();
        }

        pPAL->write(palRequest, beginAt);
      }

      finishedAt = MAX(finishedAt, beginAt);
    }
  }

  // Exclude CPU operation when initializing
  if (sendToPAL) {
    tick = finishedAt;
    tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::WRITE_INTERNAL);
  }

  // GC if needed
  // I assumed that init procedure never invokes GC
  static float gcThreshold = conf.readFloat(CONFIG_FTL, FTL_GC_THRESHOLD_RATIO);

  if (freeBlockRatio() < gcThreshold) {
    if (!sendToPAL) {
      panic("ftl: GC triggered while in initialization");
    }

    std::vector<uint32_t> list;
    uint64_t beginAt = tick;

    selectVictimBlock(list, beginAt);

    debugprint(LOG_FTL_PAGE_MAPPING,
               "GC   | On-demand | %u blocks will be reclaimed", list.size());

    doGarbageCollection(list, beginAt);

    debugprint(LOG_FTL_PAGE_MAPPING,
               "GC   | Done | %" PRIu64 " - %" PRIu64 " (%" PRIu64 ")", tick,
               beginAt, beginAt - tick);

    stat.gcCount++;
    stat.reclaimedBlocks += list.size();
  }
}

void PageMapping::trimInternal(Request &req, uint64_t &tick) {
  // Peek only — do not call accessCMT.  A miss would load the doomed LPN
  // into the cache (and possibly dirty-evict a useful victim) just so we
  // can erase it on the next line.
  auto *mappingData = getLiveMapping(req.lpn);

  // Scan every sub-page, not just index 0: with random I/O tweak enabled a
  // superpage can be partially mapped, so index 0 alone does not decide it.
  bool hasMappingData = false;
  if (mappingData != nullptr) {
    for (uint32_t idx = 0; idx < bitsetSize; idx++) {
      if (mappingData->at(idx).first < param.totalPhysicalBlocks) {
        hasMappingData = true;
        break;
      }
    }
  }

  if (hasMappingData) {
    if (bRandomTweak) {
      pDRAM->read(mappingData, 8 * req.ioFlag.count(), tick);
    }
    else {
      pDRAM->read(mappingData, 8, tick);
    }

    // Do trim
    for (uint32_t idx = 0; idx < bitsetSize; idx++) {
      auto &mapping = mappingData->at(idx);

      // Skip sub-pages that were never mapped — invalidating them would look
      // up a block index of totalPhysicalBlocks and panic.
      if (mapping.first >= param.totalPhysicalBlocks) {
        continue;
      }

      auto block = blocks.find(mapping.first);

      if (block == blocks.end()) {
        panic("Block is not in use");
      }

      block->second.invalidate(mapping.second, idx);
    }

    // Remove from CMT — this LPN is now invalid
    cmtErase(req.lpn);
    // Remove from GMT
    table.erase(req.lpn);

    tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::TRIM_INTERNAL);
  }
}

void PageMapping::eraseInternal(PAL::Request &req, uint64_t &tick) {
  static uint64_t threshold =
      conf.readUint(CONFIG_FTL, FTL_BAD_BLOCK_THRESHOLD);
  auto block = blocks.find(req.blockIndex);

  // Sanity checks
  if (block == blocks.end()) {
    panic("No such block");
  }

  if (block->second.getValidPageCount() != 0) {
    panic("There are valid pages in victim block");
  }

  // Erase block
  block->second.erase();

  pPAL->erase(req, tick);

  // Check erase count
  uint32_t erasedCount = block->second.getEraseCount();

  if (erasedCount < threshold) {
    // Reverse search
    auto iter = freeBlocks.end();

    while (true) {
      iter--;

      if (iter->getEraseCount() <= erasedCount) {
        // emplace: insert before pos
        iter++;

        break;
      }

      if (iter == freeBlocks.begin()) {
        break;
      }
    }

    // Insert block to free block list
    freeBlocks.emplace(iter, std::move(block->second));
    nFreeBlocks++;
  }

  // Remove block from block list
  blocks.erase(block);

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::ERASE_INTERNAL);
}

float PageMapping::calculateWearLeveling() {
  uint64_t totalEraseCnt = 0;
  uint64_t sumOfSquaredEraseCnt = 0;
  uint64_t numOfBlocks = param.totalLogicalBlocks;
  uint64_t eraseCnt;

  for (auto &iter : blocks) {
    eraseCnt = iter.second.getEraseCount();
    totalEraseCnt += eraseCnt;
    sumOfSquaredEraseCnt += eraseCnt * eraseCnt;
  }

  // freeBlocks is sorted
  // Calculate from backward, stop when eraseCnt is zero
  for (auto riter = freeBlocks.rbegin(); riter != freeBlocks.rend(); riter++) {
    eraseCnt = riter->getEraseCount();

    if (eraseCnt == 0) {
      break;
    }

    totalEraseCnt += eraseCnt;
    sumOfSquaredEraseCnt += eraseCnt * eraseCnt;
  }

  if (sumOfSquaredEraseCnt == 0) {
    return -1;  // no meaning of wear-leveling
  }

  return (float)totalEraseCnt * totalEraseCnt /
         (numOfBlocks * sumOfSquaredEraseCnt);
}

void PageMapping::calculateTotalPages(uint64_t &valid, uint64_t &invalid) {
  valid = 0;
  invalid = 0;

  for (auto &iter : blocks) {
    valid += iter.second.getValidPageCount();
    invalid += iter.second.getDirtyPageCount();
  }
}

void PageMapping::getStatList(std::vector<Stats> &list, std::string prefix) {
  Stats temp;

  temp.name = prefix + "page_mapping.gc.count";
  temp.desc = "Total GC count";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.gc.reclaimed_blocks";
  temp.desc = "Total reclaimed blocks in GC";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.gc.superpage_copies";
  temp.desc = "Total copied valid superpages during GC";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.gc.page_copies";
  temp.desc = "Total copied valid pages during GC";
  list.push_back(temp);

  // For the exact definition, see following paper:
  // Li, Yongkun, Patrick PC Lee, and John Lui.
  // "Stochastic modeling of large-scale solid-state storage systems: analysis,
  // design tradeoffs and optimization." ACM SIGMETRICS (2013)
  temp.name = prefix + "page_mapping.wear_leveling";
  temp.desc = "Wear-leveling factor";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.policy";
  temp.desc = "Active CMT replacement policy (0 = LRU, 1 = LFU)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.hits";
  temp.desc = "User mapping cache hits (CMT, excludes warm-up)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.misses";
  temp.desc = "User mapping cache misses (CMT, excludes warm-up)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.hit_rate";
  temp.desc = "User mapping cache hit rate % (CMT, excludes GC and warm-up)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.evictions";
  temp.desc = "Total CMT evictions (excludes warm-up)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.dirty_evictions";
  temp.desc = "CMT dirty evictions (required write-back to GMT)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.writebacks";
  temp.desc = "Total GMT write-back operations";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.gc_hits";
  temp.desc = "GC-triggered mapping cache hits (CMT)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.gc_misses";
  temp.desc = "GC-triggered mapping cache misses (CMT)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.capacity";
  temp.desc = "CMT capacity (max entries in cache)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.entry_bytes";
  temp.desc = "Mapping bytes held per CMT entry (8 B per sub-page mapping)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.capacity_bytes";
  temp.desc = "Effective CMT size in bytes (capacity * entry_bytes)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.occupancy";
  temp.desc = "CMT occupancy at end of simulation (entries used)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.prefetch_insertions";
  temp.desc = "Speculative CMT entries inserted by spatial prefetch";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.prefetch_hits";
  temp.desc = "Prefetched entries hit at least once before eviction";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.prefetch_evicted_unused";
  temp.desc = "Prefetched entries evicted without ever being hit (wasted)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.prefetch_triggers";
  temp.desc = "Number of times a demand miss initiated a prefetch window";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.prefetch_accuracy_percent";
  temp.desc = "Prefetch Accuracy (%)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.prefetch_pollution_percent";
  temp.desc = "Prefetch Pollution (%)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.prefetch_coverage_percent";
  temp.desc = "Prefetch Coverage (%)";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.cmt.prefetch_avg_batch_size";
  temp.desc = "Average valid LPNs fetched per prefetch window";
  list.push_back(temp);
}

void PageMapping::getStatValues(std::vector<double> &values) {
  values.push_back(stat.gcCount);
  values.push_back(stat.reclaimedBlocks);
  values.push_back(stat.validSuperPageCopies);
  values.push_back(stat.validPageCopies);
  values.push_back(calculateWearLeveling());

  // User-facing hit rate (excludes GC-triggered accesses)
  uint64_t totalLookups = stat.cmtHits + stat.cmtMisses;
  double hitRate = totalLookups > 0
      ? (double)stat.cmtHits / (double)totalLookups * 100.0
      : 0.0;

  values.push_back((double)cmtPolicy);
  values.push_back((double)stat.cmtHits);
  values.push_back((double)stat.cmtMisses);
  values.push_back(hitRate);
  values.push_back((double)stat.cmtEvictions);
  values.push_back((double)stat.cmtDirtyEvictions);
  values.push_back((double)stat.cmtWritebacks);
  values.push_back((double)stat.cmtGCHits);
  values.push_back((double)stat.cmtGCMisses);
  values.push_back((double)cmtCapacity);
  values.push_back((double)cmtEntryBytes);
  values.push_back((double)(cmtCapacity * cmtEntryBytes));
  values.push_back((double)cmtSize());

  double prefetchAccuracy = stat.cmtPrefetchInsertions > 0
      ? (double)stat.cmtPrefetchHits / (double)stat.cmtPrefetchInsertions * 100.0
      : 0.0;
  
  double prefetchPollution = stat.cmtEvictions > 0
      ? (double)stat.cmtPrefetchEvictedUnused / (double)stat.cmtEvictions * 100.0
      : 0.0;
  
  double prefetchCoverage = stat.cmtHits > 0
      ? (double)stat.cmtPrefetchHits / (double)stat.cmtHits * 100.0
      : 0.0;
  
  double prefetchAvgBatch = stat.cmtPrefetchTriggers > 0
      ? (double)stat.cmtPrefetchInsertions / (double)stat.cmtPrefetchTriggers
      : 0.0;

  values.push_back((double)stat.cmtPrefetchInsertions);
  values.push_back((double)stat.cmtPrefetchHits);
  values.push_back((double)stat.cmtPrefetchEvictedUnused);
  values.push_back((double)stat.cmtPrefetchTriggers);
  values.push_back(prefetchAccuracy);
  values.push_back(prefetchPollution);
  values.push_back(prefetchCoverage);
  values.push_back(prefetchAvgBatch);
}

void PageMapping::resetStatValues() {
  memset(&stat, 0, sizeof(stat));
}

}  // namespace FTL

}  // namespace SimpleSSD
