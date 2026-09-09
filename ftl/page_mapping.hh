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

#ifndef __FTL_PAGE_MAPPING__
#define __FTL_PAGE_MAPPING__

#include <cinttypes>
#include <list>
#include <unordered_map>
#include <vector>

#include "ftl/abstract_ftl.hh"
#include "ftl/common/block.hh"
#include "ftl/config.hh"
#include "ftl/ftl.hh"
#include "pal/pal.hh"

namespace SimpleSSD {

namespace FTL {

class PageMapping : public AbstractFTL {
 private:
  PAL::PAL *pPAL;

  ConfigReader &conf;

  std::unordered_map<uint64_t, std::vector<std::pair<uint32_t, uint32_t>>>
      table;
  std::unordered_map<uint32_t, Block> blocks;
  std::list<Block> freeBlocks;
  uint32_t nFreeBlocks;  // For some libraries which std::list::size() is O(n)
  std::vector<uint32_t> lastFreeBlock;
  Bitset lastFreeBlockIOMap;
  uint32_t lastFreeBlockIndex;

  bool bReclaimMore;
  bool bRandomTweak;
  uint32_t bitsetSize;

  // ── CMT (Cached Mapping Table) ─────────────────────────────
  // A size-limited cache of mapping entries sitting on top of the GMT
  // (`table`).  Models the SRAM mapping cache inside a real SSD controller,
  // as described in the DFTL paper (Gupta et al., ASPLOS '09).
  //
  // Two replacement policies are compiled in and selected at run time by the
  // `CMTPolicy` config key.  Only the structures belonging to the active
  // policy are ever populated; every operation that touches the cache goes
  // through accessCMT() / cmtErase() / cmtSize() so the two policies can
  // never fall out of sync.

  CMT_POLICY cmtPolicy;  // which replacement policy is active

  uint64_t cmtCapacity;  // max entries in cache — set in constructor
  uint64_t cmtEntryBytes;        // bytes of mapping data held per CMT entry
  uint64_t cmtMissLatency;       // NAND read latency on CMT miss (ps)
  uint64_t cmtWriteBackLatency;  // NAND program latency on dirty eviction (ps)

  bool cmtWindowFill;
  uint64_t cmtWindowSize;

  // ── LRU policy state ───────────────────────────────────────
  // Evicts the entry that was accessed least recently.
  struct CMTEntry {
    std::vector<std::pair<uint32_t, uint32_t>> mapping;  // physical (block, page)
    bool dirty;  // true if modified while in cache (needs write-back on eviction)
    bool fillOrigin;
    bool fillUnused;
  };

  // LRU ordering: front = most recently used, back = least recently used
  std::list<uint64_t> cmtOrder;

  // CMT store: LPN → {CMTEntry, iterator into cmtOrder for O(1) LRU update}
  std::unordered_map<uint64_t,
    std::pair<CMTEntry, std::list<uint64_t>::iterator>> cmt;

  // ── LFU policy state ───────────────────────────────────────
  // Evicts the entry with the fewest lifetime accesses, breaking ties by
  // recency.  O(1) frequency-bucket algorithm (Shah, Mitra, Matani 2010):
  //   cmtFreqBuckets[f] = LPNs at frequency f, MRU at front.
  //   cmtMinFreq        = smallest occupied frequency = eviction bucket.
  // On HIT:  move LPN from bucket[f] → bucket[f+1].
  // On MISS: evict bucket[cmtMinFreq].back(), insert new entry at freq 1.
  struct CMTEntryLFU {
    std::vector<std::pair<uint32_t, uint32_t>> mapping;
    bool     dirty;    // needs write-back on eviction if true
    uint64_t freq;     // lifetime hit count — never resets while in cache
    std::list<uint64_t>::iterator listIt;  // O(1) removal from freq bucket
    bool     fillOrigin;
    bool     fillUnused;
  };

  // frequency → list of LPNs at that freq (front=MRU for tie-break)
  std::unordered_map<uint64_t, std::list<uint64_t>> cmtFreqBuckets;

  // main LFU store: LPN → CMTEntryLFU
  std::unordered_map<uint64_t, CMTEntryLFU> cmtLFU;

  // always points at the eviction-candidate bucket
  uint64_t cmtMinFreq;

  // ── CMT operations ─────────────────────────────────────────
  // Call accessCMT() instead of table.find() for all reads/writes.  When
  // `allocate` is false a true miss returns nullptr instead of creating a
  // mapping, so read/trim of a never-written LPN does not pollute the cache.
  std::vector<std::pair<uint32_t, uint32_t>> *accessCMT(uint64_t lpn,
                                                        bool isWrite,
                                                        uint64_t &tick,
                                                        bool isGC = false,
                                                        bool allocate = true);

  std::vector<std::pair<uint32_t, uint32_t>> *accessCMT_LRU(uint64_t lpn,
                                                            bool isWrite,
                                                            uint64_t &tick,
                                                            bool isGC,
                                                            bool allocate);

  std::vector<std::pair<uint32_t, uint32_t>> *accessCMT_LFU(uint64_t lpn,
                                                            bool isWrite,
                                                            uint64_t &tick,
                                                            bool isGC,
                                                            bool allocate);

  void evictOneLRUVictim(uint64_t &tick, bool chargeWriteBack = true);
  void evictOneLFUVictim(uint64_t &tick, bool chargeWriteBack = true);

  // CMT window fill: one NAND translation-page read already paid
  // CMTMissLatency; install neighboring GMT mappings into the CMT.
  // A separate translation-page buffer (DFTL GTD/TP cache) is not modeled —
  // mappings are installed directly into the CMT.
  bool cmtContains(uint64_t lpn) const;
  std::vector<uint64_t> collectFillCandidates(uint64_t lpn) const;
  void evictForFillBatch(size_t batchSize, uint64_t &tick);
  uint64_t insertFillBatchLRU(const std::vector<uint64_t> &candidates);
  uint64_t insertFillBatchLFU(const std::vector<uint64_t> &candidates);
  void chargeWindowFillDRAM(uint64_t nEntries, uint64_t &tick);
  std::vector<std::pair<uint32_t, uint32_t>> *cmtMappingOf(uint64_t lpn);
  uint64_t countResidentFills() const;

  // Drop one LPN from whichever cache is active (no write-back — callers use
  // this when the mapping is being destroyed, e.g. trim and format).
  void cmtErase(uint64_t lpn);

  // After an LFU bucket is removed, point cmtMinFreq at the smallest
  // remaining frequency (or 0 if the cache is empty).
  void repairLFUMinFreq();

  // Mapping currently in force for an LPN: the CMT copy if resident (may be
  // dirty and ahead of GMT), otherwise the GMT entry.  No cache mutation,
  // no stats, no latency — for destroy paths (trim/format) that must not
  // load a doomed LPN into the cache.
  std::vector<std::pair<uint32_t, uint32_t>> *getLiveMapping(uint64_t lpn);

  // Number of entries currently resident in the active cache.
  uint64_t cmtSize() const;

  // Flush all dirty CMT entries back to GMT and clear the cache.
  // Called at destruction to keep GMT coherent after simulation.
  void flushCMT();

  // Zero only the CMT counters.  Called once warm-up finishes so the reported
  // hit rate describes the measured workload rather than the prefill.
  void resetCMTStats();

  struct {
    uint64_t gcCount;
    uint64_t reclaimedBlocks;
    uint64_t validSuperPageCopies;
    uint64_t validPageCopies;
    // ── NEW: CMT statistics ────────────────────
    uint64_t cmtHits;           // user lookups served from CMT (fast path)
    uint64_t cmtMisses;         // user lookups that required reading from GMT
    uint64_t cmtEvictions;      // total entries evicted from CMT
    uint64_t cmtDirtyEvictions; // evictions that required write-back to GMT
    uint64_t cmtWritebacks;     // total write-back operations to GMT
    uint64_t cmtGCHits;         // GC-triggered lookups served from CMT
    uint64_t cmtGCMisses;       // GC-triggered lookups that required GMT read
    uint64_t cmtFillInsertions; // speculative entries inserted
    uint64_t cmtFillHits;       // window-filled entries later confirmed useful
    uint64_t cmtFillEvictedUnused; // window-filled entries evicted having never been hit
    uint64_t cmtFillTriggers;   // demand misses that initiated a window-fill batch
  } stat;

  float freeBlockRatio();
  uint32_t convertBlockIdx(uint32_t);
  uint32_t getFreeBlock(uint32_t);
  uint32_t getLastFreeBlock(Bitset &);
  void calculateVictimWeight(std::vector<std::pair<uint32_t, float>> &,
                             const EVICT_POLICY, uint64_t);
  void selectVictimBlock(std::vector<uint32_t> &, uint64_t &);
  void doGarbageCollection(std::vector<uint32_t> &, uint64_t &);

  float calculateWearLeveling();
  void calculateTotalPages(uint64_t &, uint64_t &);

  void readInternal(Request &, uint64_t &);
  void writeInternal(Request &, uint64_t &, bool = true);
  void trimInternal(Request &, uint64_t &);
  void eraseInternal(PAL::Request &, uint64_t &);

 public:
  PageMapping(ConfigReader &, Parameter &, PAL::PAL *, DRAM::AbstractDRAM *);
  ~PageMapping();

  bool initialize() override;

  void read(Request &, uint64_t &) override;
  void write(Request &, uint64_t &) override;
  void trim(Request &, uint64_t &) override;

  void format(LPNRange &, uint64_t &) override;

  Status *getStatus(uint64_t, uint64_t) override;

  void getStatList(std::vector<Stats> &, std::string) override;
  void getStatValues(std::vector<double> &) override;
  void resetStatValues() override;
};

}  // namespace FTL

}  // namespace SimpleSSD

#endif
