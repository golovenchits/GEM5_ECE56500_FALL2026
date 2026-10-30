#ifndef __CPU_O3_WIB_HH__
#define __CPU_O3_WIB_HH__

#include <list>
#include <string>
#include <utility>
#include <vector>

#include "base/statistics.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/reg_class.hh"

namespace gem5
{

struct BaseO3CPUParams;

namespace o3
{

class CPU;

struct DerivO3CPUParams;

class WIB
{
  public:
    typedef std::pair<RegIndex, RegIndex> UnmapInfo;
    typedef typename std::list<DynInstPtr>::iterator InstIt;

    /** Possible WIB statuses. */
    enum Status
    {
        Running,
        Idle,
        WIBSquashing
    };

  private:
    /** WIB status. */
    Status wibStatus;

  public:
    /** WIB constructor.
     *  @param _cpu   The cpu object pointer.
     *  @param params The cpu params including several WIB-specific parameters.
     */
    WIB(CPU *_cpu, const BaseO3CPUParams &params);

    std::string name() const;

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;

    /** Takes over another CPU's thread. */
    void takeOverFrom();

    /** Returns the bank the next dispatched instruction maps to and advances
     *  the round-robin allocation pointer. Must be called once for every
     *  dispatched instruction (not just ones that enter the WIB) so that
     *  entries are interleaved across banks in program order.
     */
    unsigned allocateBank();

    /** Function to insert an instruction into the WIB. Note that whatever
     *  calls this function must ensure that there is enough space within the
     *  bank for the new instruction.
     *  @param inst The instruction being inserted into the WIB.
     */
    void insertInst(const DynInstPtr &inst);

    /** Returns pointer to the head instruction within the WIB.  There is
     *  no guarantee as to the return value if the WIB is empty.
     *  @retval Pointer to the DynInst that is at the head of the WIB.
     */
//    DynInstPtr readHeadInst();

    /** Returns a pointer to the instruction with the given sequence if it is
     *  in the WIB.
     */
    DynInstPtr findInst(InstSeqNum squash_inst);

    /** Returns pointer to the tail instruction within the WIB.  There is
     *  no guarantee as to the return value if the WIB is empty.
     *  @retval Pointer to the DynInst that is at the tail of the WIB.
     */
//    DynInstPtr readTailInst();

    /** Retires the head instruction, removing it from the WIB. */
   void retireHead(unsigned wib_idx);

    /** Is the oldest instruction ready. */
//    bool isHeadReady();

    /** Returns the number of WIB banks. */
    unsigned getNumBanks() const
    { return numBanks; }

    /** Returns the number of total free entries in the WIB. */
    unsigned numFreeEntries();

    /** Returns the number of free entries in a specific bank. */
    unsigned numFreeEntries(unsigned bank);

    /** Returns if the WIB is full. */
    bool isFull()
    { return numInstsInWIB == numEntries; }

    /** Returns if a specific bank is full. */
    bool isFull(unsigned bank) const
    { return bankEntries[bank] == maxBankEntries; }

    /** Returns if the WIB is empty. */
    bool isEmpty() const
    { return numInstsInWIB == 0; }

    /** Returns if a specific bank is empty. */
    bool isEmpty(unsigned bank) const
    { return bankEntries[bank] == 0; }

    /** Executes the squash, marking squashed instructions in every bank. */
    void doSquash();

    /** Squashes all instructions younger than the given sequence number in
     *  every bank and resets each bank's tail.
     */
    void squash(InstSeqNum squash_num);

    /** Updates a bank's head with its new oldest instruction. */
    void updateHead(unsigned bank);

    /** Updates a bank's tail with its new youngest instruction. */
    void updateTail(unsigned bank);

    /** Updates the bank priority order after a reinsertion attempt. A bank
     *  that had an instruction but could not reinsert it stays at the highest
     *  priority; a bank that reinserted or had nothing goes to the lowest.
     *  @param bank        The bank that was examined.
     *  @param reinserted  Whether the bank reinserted an instruction.
     *  @param had_inst    Whether the bank had an eligible instruction.
     */
    void updateBankPriority(unsigned bank, bool reinserted, bool had_inst);

    /** Switches between reading the even and odd banks. Called once per
     *  cycle, since each bank can only be read every other cycle.
     */
    void switchBankSet()
    { readOddBanks = !readOddBanks; }

    /** Reads the PC of the oldest head instruction. */
//    uint64_t readHeadPC();

    /** Reads the next PC of the oldest head instruction. */
//    uint64_t readHeadNextPC();

    /** Reads the sequence number of the oldest head instruction. */
//    InstSeqNum readHeadSeqNum();

    /** Reads the PC of the youngest tail instruction. */
//    uint64_t readTailPC();

    /** Reads the sequence number of the youngest tail instruction. */
//    InstSeqNum readTailSeqNum();

    /** Checks if the WIB is still in the process of squashing instructions.
     *  @retval Whether or not the WIB is done squashing.
     */
    bool isDoneSquashing() const
    { return doneSquashing; }

    /** This is more of a debugging function than anything.  Use
     *  numInstsInWIB to get the instructions in the WIB unless you are
     *  double checking that variable.
     */
    int countInsts();

    /** Debugging function to count the instructions in a specific bank. Use
     *  bankEntries unless you are double checking that variable.
     */
    size_t countInsts(unsigned bank);

  private:
    /** Reset the WIB state */
    void resetState();

    /** Pointer to the CPU. */
    CPU *cpu;

    /** Total number of WIB entries across all banks. */
    unsigned numEntries;

    /** WIB List of Instructions, one program-ordered list per bank. */
    std::vector<std::list<DynInstPtr>> instBanks;

    /** Number of instructions that can be squashed in a single cycle. */
    unsigned squashWidth;

    unsigned wibToIQWidth;

    /** Number of banks (twice wibToIQWidth in the paper). */
    unsigned numBanks;

    /** Number of entries in each bank (numEntries / numBanks). */
    unsigned maxBankEntries;

    /** Number of instructions currently in each bank. */
    std::vector<unsigned> bankEntries;

    /** Idx the next dispatched instruction is allocated to. */
    unsigned allocIdx;

    /** Whether the odd (true) or even (false) banks are read this cycle. */
    bool readOddBanks;

    /** Bank indices ordered from highest to lowest reinsertion priority. */
    std::list<unsigned> bankPriority;

  public:
    /** Per-bank iterator pointing to the last instruction in that bank.
     *  This may at times be invalid (ie when the bank is empty), however it
     *  should never be incorrect.
     */

    /** Per-bank iterator pointing to the first instruction in that bank. */
    std::vector<InstIt> heads;
    std::vector<InstIt> tails;


  private:
    /** Per-bank iterator used for walking through the list of instructions
     *  when squashing.  Used so that there is persistent state between
     *  cycles; when squashing, the instructions are marked as squashed but
     *  not immediately removed, meaning the tail iterator remains the same
     *  before and after a squash.
     *  This will always be set to instBanks[bank].end() if it is invalid.
     */
    std::vector<InstIt> squashIt;

  public:
    /** Number of instructions in the WIB. */
    int numInstsInWIB;

    /** Dummy instruction returned if there are no insts left. */
    DynInstPtr dummyInst;

  private:
    /** The sequence number of the squashed instruction. */
    InstSeqNum squashedSeqNum;

    /** Is the WIB done squashing. */
    bool doneSquashing;


    struct WIBStats : public statistics::Group
    {
        WIBStats(statistics::Group *parent);

        // The number of wib_reads
        statistics::Scalar reads;
        // The number of wib_writes
        statistics::Scalar writes;
    } stats;

    /// BV ARRAY ///
  private:

    int maxNumCols;

    std::vector<std::vector<bool>> bitVecArr;
    std::vector<int> freeCols;
    std::vector<bool> colDone;

  public:
    int allocateLoadCol(const DynInstPtr &load);
    void loadColDone(int col);
    void freeLoadCol(int col);
};

} // namespace o3
} // namespace gem5

#endif //__CPU_O3_WIB_HH__
