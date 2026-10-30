
#include "cpu/o3/wib.hh"

#include <list>

#include "base/logging.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/o3/limits.hh"
#include "debug/Fetch.hh"
// #include "debug/WIB.hh"
#include "params/BaseO3CPU.hh"
#include "cpu/o3/cpu.hh"

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

int WIB::allocateLoadCol(const DynInstPtr &load){
    if (freeCols.empty()) {
        panic("No free columns available in WIB");
        // TODO: maybe need to stall instead of panicking
    }
    int new_col = freeCols.back();
    freeCols.pop_back();
    colDone[new_col] = false;

    bitVecArr[new_col].assign(numEntries, false);
    // bitVecArr[new_col][load->wibIdx] = true; // Don't need to do this?
    return new_col;
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


void
WIB::insertInst(const DynInstPtr &inst)
{
    assert(inst);
    assert(!isFull());

    unsigned bank = allocIdx % numBanks;
    inst->wibIdx = allocIdx;
    allocIdx = (allocIdx + 1) % numEntries;

    // Dispatch is in program order, so each bank stays in program order
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

}
}