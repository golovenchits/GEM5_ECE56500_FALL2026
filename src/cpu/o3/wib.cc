
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
      squashWidth(params.squashWidth),
      wibToIQWidth(params.wibToIQWidth),
      numBanks(2 * params.wibToIQWidth),
      maxBankEntries(numEntries/numBanks),
      numInstsInWIB(0),
      stats(_cpu),
      maxNumCols(std::min(params.numWIBBitVectors, params.LQEntries))
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
    colState.assign(maxNumCols, ColState::Free);
    colOwnerSeqNum.assign(maxNumCols, 0);
    bitVecArr.assign(maxNumCols, std::vector<bool>(numEntries, false));
    // orBitVec.assign(numEntries, false);
    // reinsertionInsns.clear();
    bankPriorities.assign(numBanks, 0);
}

// void
// WIB::markColDone(int col)
// {
//     assert(colState[col] == ColState::Waiting);
//     colState[col] = ColState::Done;
// }

std::list<DynInstPtr>
WIB::getReinsertInsts(bool isOdd, int numAvailIQEntries) // 
{
    std::vector<bool> orBitVec(numEntries, false); // has ROB entries number of bools
    std::list<DynInstPtr> reinsertionInsts;

    for (int col = 0; col < maxNumCols; col++) {
        if (colState[col] == ColState::Done) {
            for (int entry = 0; entry < numEntries; entry++) {
                orBitVec[entry] |= bitVecArr[col][entry];
            }
        };
    };

    // Create a vector of bank IDs
    std::vector<int> bankIdxs (numBanks);
    std::iota(bankIdxs.begin(), bankIdxs.end(), 0);

    // Sort the indices based on the bank priorities
    std::sort(bankIdxs.begin(), bankIdxs.end(), [&](int i, int j) {
        return bankPriorities[i] < bankPriorities[j];
    });

    // for (int bank = isOdd; bank < numBanks; bank += 2) {
    for (auto bIt = bankIdxs.begin(); bIt < bankIdxs.end(); bIt++) {
        int bank = *bIt;
        if (bank % 2 != isOdd) continue;
        if (bankHasReinsertable(bank) == true) {
            if (numAvailIQEntries - reinsertionInsts.size() > 0) {
                DynInstPtr wibBankEntry = instBanks[bankIdxs[i]][idx];
                if (wibBankEntry->isSquashed()) continue;
                reinsertionInsts.push_back(wibBankEntry);
                bankPriorities[bank] = 0;
                bitVecArr[wibBankEntry->bvaColIdx][wibBankEntry->wibIdx] = false;
            } else {
                bankPriorities[bank]++;
            }
        } else bankPriorities[bank] = 0;
    }
    
    assert(reinsertionInsts.size() <= wibToIQWidth);
    assert(numAvailIQEntries >= reinsertionInsts.size());

    // check if all dependent instructions of a load is reinserted and can free
    // the bit vector column
    freeBVAColIfReady();

    return reinsertionInsts;
}

bool
WIB::bankHasReinsertable(int bankIdx) {
    int slice_start = bank * maxBankEntries;
    for (int idx = slice_start; idx < slice_start + maxBankEntries; idx++) {
        if (orBitVec[idx] == true) return true;
    }
    return false;
}

void
WIB::freeBVAColIfReady(int col) {
    for (int col = 0; col < maxNumCols; col++) {
        if (std::none_of(bitVecArr[col].begin(), bitVecArr[col].end(), 
                [](bool v){return v;}) && (colState[col] == ColState::Done)) {
            markColFree(col);
        }
    }
}

// all priorities start at 0.
// if bank has an instruction to insert, start counting per insertion cycle.
// sort by highest priority.
// clear priority to 0 if bank has no instruction to insert or just inserted.
// need to set colstate to free after all of dependent instructions have been 
// reinserted
// above done

// ??? cycles after ???, you reinsert the instruction into the issueQ.

int
WIB::allocateLoadCol(const DynInstPtr &load)
{
    for (int col = 0; col < maxNumCols; col++){
        if (colState[col] == ColState::Free) {
            colState[col] = ColState::Waiting;
            colOwnerSeqNum[col] = load->seqNum;
            bitVecArr[col].assign(numEntries, false);
            return col;

        }
    }
    DPRINTF(WIB, "No free bit-vector for load [sn:%llu].\n",
                 load->seqNum);

    return -1;
}

void
WIB::parkInst(const DynInstPtr &inst, int col)
{
    assert(col >= 0 && col < maxNumCols);
    assert(colState[col] == ColState::Waiting);
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
WIB::squash(InstSeqNum squash_num)
{

    DPRINTF(WIB, "Starting to squash within the WIB.\n");

    // wibStatus = WIBSquashing;
    // doneSquashing = false;

    for (unsigned bank = 0; bank < numBanks; bank++){
        for (auto it = instBanks[bank].rbegin();
                it != instBanks[bank].rend() && (*it)->seqNum > squash_num;
                ++it){
                const DynInstPtr &inst = *it;
                if(inst->isInWIB()){
                    inst->clearInWIB();
                    bitVecArr[inst->bvaColIdx][inst->wibIdx] = false;
                    inst->bvaColIdx = -1;
                }
        }
    }

    for (int col = 0; col < maxNumCols; col++){
        if(colState[col] != ColState::Free && colOwnerSeqNum[col] > squash_num)
            markColFree(col);
    }
}

void
WIB::markColFree(int col)
{
    assert(colState[col] != ColState::Free);
    assert(std::none_of(bitVecArr[col].begin(), 
                        bitVecArr[col].end(),
                        [](bool v){return v;}));
    
    colState[col] = ColState::Free;
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