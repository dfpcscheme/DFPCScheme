/*******************************************************************************
* Copyright (c) 2012-2014, The Microsystems Design Labratory (MDL)
* Department of Computer Science and Engineering, The Pennsylvania State University
* All rights reserved.
* 
* This source code is part of NVMain - A cycle accurate timing, bit accurate
* energy simulator for both volatile (e.g., DRAM) and non-volatile memory
* (e.g., PCRAM). The source code is free and you can redistribute and/or
* modify it by providing that the following conditions are met:
* 
*  1) Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
* 
*  2) Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
* Author list: 
*   Matt Poremba    ( Email: mrp5060 at psu dot edu 
*                     Website: http://www.cse.psu.edu/~poremba/ )
*******************************************************************************/

#ifndef __FRFCFS_H__
#define __FRFCFS_H__

#include "src/MemoryController.h"
#include <deque>

//EDFPCscheme
#define SAMPLECOUNT 128
#define FPCCOUNT 3
#define BDICOUNT 8
#define DYNAMICWORDSIZE 8 //chars

namespace NVM {

class FRFCFS : public MemoryController
{
  public:
    FRFCFS( );
    ~FRFCFS( );

    bool IssueCommand( NVMainRequest *req );
    bool IsIssuable( NVMainRequest *request, FailReason *fail = NULL );
    bool RequestComplete( NVMainRequest * request );

    void SetConfig( Config *conf, bool createChildren = true );

    void Cycle( ncycle_t steps );

    void RegisterStats( );
    void CalculateStats( );

  private:
    NVMTransactionQueue *memQueue;

    /* Cached Configuration Variables*/
    uint64_t queueSize;

    /* Stats */
    uint64_t measuredLatencies, measuredQueueLatencies, measuredTotalLatencies;
    double averageLatency, averageQueueLatency, averageTotalLatency;
    uint64_t mem_reads, mem_writes;
    uint64_t rb_hits;
    uint64_t rb_miss;
    uint64_t starvation_precharges;
    uint64_t cpu_insts;
    uint64_t write_pauses;
	//EDFPCscheme
    
    uint64_t bit_write;
    uint64_t bit_write_before;
    double compress_ratio;
    
    uint64_t my_llabs ( int64_t x );
    uint64_t my_abs ( int x );
    bool GeneralCompress (NVMainRequest *request, uint64_t compress);
    bool Encoder (NVMainRequest *request, bool flag);
    bool GeneralEncoder (NVMainRequest *request);
    uint64_t GetChanges (NVMainRequest *request, uint32_t MLCLevels, bool DCWFlag);
    uint64_t * convertByte2Word (NVMainRequest *request, bool flag, uint64_t size, uint64_t step);//flag: false-olddata true-newdata
    bool Word2Byte (NVMainRequest *request, bool flag, uint64_t size, uint64_t comSize, uint64_t *words, uint64_t *wordPos);//flag: false-olddata true-newdata
    
    bool FPCCompress(NVMainRequest *request, uint64_t size, bool flag );
    bool BDICompress (NVMainRequest *request, uint64_t _blockSize, bool flag );
    bool DFPCCompress(NVMainRequest *request, uint64_t _blockSize);
    
    
    bool isZeroPackable ( uint64_t * values, uint64_t size);
    bool isSameValuePackable ( uint64_t * values, uint64_t size);
    uint64_t multBaseCompression ( uint64_t * values, uint64_t size, uint64_t blimit, uint64_t bsize, uint64_t *currWords, uint64_t *currWordPos, uint64_t &pos);
    
    bool StaticCompress(NVMainRequest *request, uint64_t size, bool flag );
    uint64_t FPCIdentify (NVMainRequest *request, uint64_t size);
	uint64_t BDIIdentify (NVMainRequest *request, uint64_t _blockSize);
	uint64_t Sample (NVMainRequest *request, uint64_t _blockSize);
    uint64_t ExtractPattern();
    void HeapSort(uint64_t pattern_array[], uint64_t bytes_array[],int length, int topk);
    void HeapAdjust(uint64_t pattern_array[], uint64_t bytes_array[],int pos,int nLength);
	bool DynamicCompress(NVMainRequest *request, uint64_t size, bool flag );
    
    uint64_t DynamicFPCCompress(NVMainRequest *request, uint64_t size, bool flag );
    uint64_t DynamicBDICompress(NVMainRequest *request, uint64_t _blockSize, bool flag );
    
	uint64_t BDIpatterncounter;
    bool sample_flag;
    uint32_t pattern_num;
	
	uint64_t granularities;
    double threshold_factor;
	uint64_t FPCCounter[FPCCOUNT];
	uint64_t BDICounter[BDICOUNT];
	uint64_t SampleCounter[SAMPLECOUNT];
    uint64_t masks[FPCCOUNT+SAMPLECOUNT/DYNAMICWORDSIZE];
    int compressibleChars[FPCCOUNT+SAMPLECOUNT/DYNAMICWORDSIZE];
    bool special_pattern_flag[1+BDICOUNT];
    int mask_pos;
    
    bool encodeFlag;
    uint64_t compressIndex;
};

};

#endif
