
#include "cpu/o3/wib.hh"

#include <list>

#include "base/logging.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/o3/limits.hh"
#include "debug/Fetch.hh"
#include "debug/WIB.hh"
#include "params/BaseO3CPU.hh"
#include "cpu/o3/cpu.hh"
#include <algorithm>

namespace gem5
{

namespace o3
{

WIB::WIB(CPU *_cpu, const BaseO3CPUParams &params)

    : cpu(_cpu),
      numEntries(params.numROBEntries),
      wibToIQWidth(params.wibToIQWidth),
      numBanks(2 * params.wibToIQWidth),
      squashWidth(params.squashWidth),
      maxBankEntries(numEntries/numBanks),
      numInstsInWIB(0),
      maxNumCols(std::min(params.numWIBBitVectors, params.LQEntries)),
      stats(_cpu)
{
    resetState();
}



void
WIB::resetState()
{
    // Reset WIB
    tails.assign(numBanks, InstIt());
    heads.assign(numBanks, InstIt());
    allocIdx = 0;
    numInstsInWIB = 0;
    instBanks.assign(numBanks, std::list<DynInstPtr>());
    bankEntries.assign(numBanks, 0);

    // Reset BVA
    freeCols.clear();
    for (int col = maxNumCols - 1; col >= 0; col--)
        freeCols.push_back(col);
    colDone.assign(maxNumCols, false);
    bitVecArr.assign(maxNumCols, std::vector<bool>(numEntries, false));
}

int
WIB::allocateLoadCol(const DynInstPtr &load)
{
    // No free bit-vector: the miss is handled without the WIB
    if (freeCols.empty()) {
        DPRINTF(WIB, "No free bit-vector for load [sn:%llu].\n",
                load->seqNum);
        return -1;
    }

    int new_col = freeCols.back();
    freeCols.pop_back();
    colDone[new_col] = false;

    bitVecArr[new_col].assign(numEntries, false);
    return new_col;
}

void
WIB::parkInst(const DynInstPtr &inst, int col)
{
    assert(col >= 0 && col < maxNumCols);
    assert(!colDone[col]);
    assert(!bitVecArr[col][inst->wibIdx]);

    inst->bvaColIdx = col;
    inst->setInWIB();
    bitVecArr[col][inst->wibIdx] = true;

    stats.writes++;
    DPRINTF(WIB, "Parked [sn:%llu] in slot %u under bit-vector %i.\n",
            inst->seqNum, inst->wibIdx, col);
}

std::string
WIB::name() const
{
    return cpu->name() + ".wib";
}

int
WIB::countInsts()
{
    int count = 0;
    for (const auto &bank : instBanks) {
        count += bank.size();
    }
    return count;
}

// this runs at DISPATCH, for EVERY instruction, not just ones that enter the WIB
void
WIB::insertInst(const DynInstPtr &inst)
{
    assert(inst);
    assert(!isFull());

    unsigned bank = allocIdx % numBanks;
    inst->wibIdx = allocIdx;
    allocIdx = (allocIdx + 1) % numEntries;

    instBanks[bank].push_back(inst);
    ++bankEntries[bank];
    ++numInstsInWIB;

    stats.writes++;
    DPRINTF(WIB, "Allocated [sn:%llu] to slot %u (bank %u).\n",
            inst->seqNum, inst->wibIdx, bank);

}

void WIB::retireHead(unsigned wib_idx){
    stats.writes++;

    assert(!isEmpty());

    unsigned bank = wib_idx % numBanks;
    assert(!instBanks[bank].empty());
    assert(instBanks[bank].front()->wibIdx == wib_idx);

    instBanks[bank].pop_front();
    --bankEntries[bank];
    --numInstsInWIB;
}

void
WIB::takeOverFrom()
{
    resetState();
}

void
WIB::drainSanityCheck() const
{
    assert(isEmpty());
}

WIB::WIBStats::WIBStats(statistics::Group *parent)
  : statistics::Group(parent, "wib"),
    ADD_STAT(reads, statistics::units::Count::get(),
        "The number of WIB reads"),
    ADD_STAT(writes, statistics::units::Count::get(),
        "The number of WIB writes")
{
}

} // namespace o3
} // namespace gem5