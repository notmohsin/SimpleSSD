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
  // Implements a size-limited LRU cache on top of the GMT (table).
  // Simulates the SRAM mapping cache inside a real SSD controller.

  struct CMTEntry {
    std::vector<std::pair<uint32_t, uint32_t>> mapping;  // physical (block, page)
    bool dirty;  // true if modified while in cache (needs write-back on eviction)
  };

  uint64_t cmtCapacity;  // max entries in cache — set in constructor
  uint64_t cmtMissLatency;       // NAND read latency on CMT miss (ps)
  uint64_t cmtWriteBackLatency;  // NAND program latency on dirty eviction (ps)

  // LRU ordering: front = most recently used, back = least recently used
  std::list<uint64_t> cmtOrder;

  // CMT store: LPN → {CMTEntry, iterator into cmtOrder for O(1) LRU update}
  std::unordered_map<uint64_t,
    std::pair<CMTEntry, std::list<uint64_t>::iterator>> cmt;

  // CMT access function — call instead of table.find() for all reads/writes
  std::vector<std::pair<uint32_t, uint32_t>> &accessCMT(uint64_t lpn,
                                                          bool isWrite,
                                                          uint64_t &tick,
                                                          bool isGC = false);

  // Flush all dirty CMT entries back to GMT and clear the cache.
  // Called at destruction to keep GMT coherent after simulation.
  void flushCMT();

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
