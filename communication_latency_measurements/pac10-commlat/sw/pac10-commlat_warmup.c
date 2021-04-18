//
// Copyright (c) 2017, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <uuid/uuid.h>
#include <opae/fpga.h>
#include <stdlib.h> 
#include <unistd.h>

// #include <linux/module.h>
// #include <linux/kernel.h>
// #include <linux/init.h> 
// #include <linux/hardirq.h>
// #include <linux/preempt.h>
// #include <linux/sched.h>
// State from the AFU's JSON file, extracted using OPAE's afu_json_mgr script
#include "afu_json_info.h"

#define CACHELINE_BYTES 64
#define CL(x) ((x) * CACHELINE_BYTES)


//
// Search for an accelerator matching the requested UUID and connect to it.
//
static fpga_handle connect_to_accel(const char *accel_uuid)
{
    fpga_properties filter = NULL;
    fpga_guid guid;
    fpga_token accel_token;
    uint32_t num_matches;
    fpga_handle accel_handle;
    fpga_result r;

    // Don't print verbose messages in ASE by default
    setenv("ASE_LOG", "0", 0);

    // Set up a filter that will search for an accelerator
    fpgaGetProperties(NULL, &filter);
    fpgaPropertiesSetObjectType(filter, FPGA_ACCELERATOR);

    // Add the desired UUID to the filter
    uuid_parse(accel_uuid, guid);
    fpgaPropertiesSetGUID(filter, guid);

    // Do the search across the available FPGA contexts
    num_matches = 1;
    fpgaEnumerate(&filter, 1, &accel_token, 1, &num_matches);

    // Not needed anymore
    fpgaDestroyProperties(&filter);

    if (num_matches < 1)
    {
        fprintf(stderr, "Accelerator %s not found!\n", accel_uuid);
        return 0;
    }

    // Open accelerator
    r = fpgaOpen(accel_token, &accel_handle, 0);
    assert(FPGA_OK == r);

    // Done with token
    fpgaDestroyToken(&accel_token);

    return accel_handle;
}


//
// Allocate a buffer in I/O memory, shared with the FPGA.
//
static volatile void* alloc_buffer(fpga_handle accel_handle,
                                   ssize_t size,
                                   uint64_t *wsid,
                                   uint64_t *io_addr)
{
    fpga_result r;
    volatile void* buf;

    r = fpgaPrepareBuffer(accel_handle, size, (void*)&buf, wsid, 0);
    if (FPGA_OK != r) return NULL;

    // Get the physical address of the buffer in the accelerator
    r = fpgaGetIOAddress(accel_handle, *wsid, io_addr);
    assert(FPGA_OK == r);

    return buf;
}


int main(int argc, char *argv[])
{
    fpga_handle accel_handle;
    volatile char *buf;
    uint64_t wsid;
    uint64_t buf_pa;

    // Find and connect to the accelerator
    accel_handle = connect_to_accel(AFU_ACCEL_UUID);

    // Allocate a single page memory buffer
    buf = (volatile char*)alloc_buffer(accel_handle, getpagesize(),
                                       &wsid, &buf_pa);
    assert(NULL != buf);


    // Set the low byte of the shared buffer to 0.  The FPGA will write
    // a non-zero value to it.
    buf[0] = 0;

    // variables for time measurement
    unsigned long flags;
    unsigned int first_time = 0;
    printf("1 10 100 1000 10000 100000 100000\n");
    unsigned int loop =0;
    for(loop = 1; loop<=100000; loop=loop*10){
        unsigned long long accumulated_time = 0;
        int trial;
        for(trial = 0; trial <100; ++trial){
            uint64_t start=0, end=0;
            unsigned cycles_low=0, cycles_high=0, cycles_low1=0, cycles_high1=0;

            asm volatile ("CPUID\n\t"
            "RDTSC\n\t"
            "mov %%edx, %0\n\t"
            "mov %%eax, %1\n\t": "=r" (cycles_high), "=r" (cycles_low)::
            "%rax", "%rbx", "%rcx", "%rdx");
            asm volatile("RDTSCP\n\t"
            "mov %%edx, %0\n\t"
            "mov %%eax, %1\n\t"
            "CPUID\n\t": "=r" (cycles_high1), "=r" (cycles_low1):: "%rax",
            "%rbx", "%rcx", "%rdx");

            // preempt_disable();
            // raw_local_irq_save(flags);
            asm volatile ("CPUID\n\t"
            "RDTSC\n\t"
            "mov %%edx, %0\n\t"
            "mov %%eax, %1\n\t": "=r" (cycles_high), "=r"
            (cycles_low):: "%rax", "%rbx", "%rcx", "%rdx");

            // Tell the accelerator the address of the buffer using cache line
            // addresses.  The accelerator will respond by writing to the buffer.
            fpgaWriteMMIO64(accel_handle, 0, 0, buf_pa / CL(1));
            int i =0;
            for(i=0;i<1;++i){
                // Spin, waiting for the value in memory to change to something non-zero.
                while (0 == buf[0])
                {
                    // A well-behaved program would use _mm_pause(), nanosleep() or
                    // equivalent to save power here.
                };
                buf[0] = 0;
                fpgaWriteMMIO64(accel_handle, 0, 0, buf_pa / CL(1));
            }
            asm volatile("RDTSCP\n\t"
            "mov %%edx, %0\n\t"
            "mov %%eax, %1\n\t"
            "CPUID\n\t": "=r" (cycles_high1), "=r"
            (cycles_low1):: "%rax", "%rbx", "%rcx", "%rdx");

            // raw_local_irq_restore(flags);
            // preempt_enable();
            start = ( ((uint64_t)cycles_high << 32) | cycles_low );
            end = ( ((uint64_t)cycles_high1 << 32) | cycles_low1 );
            accumulated_time += (end - start);
            if(loop == 1 && trial ==0){
                first_time = end - start;
            }
            usleep(loop);
        }
        printf("%lld ",accumulated_time/100);
    }
    printf("\n first time: %d",first_time);
    // Print the string written by the FPGA
    // printf("clock cycles:%llu", end-start);

    // Done
    fpgaReleaseBuffer(accel_handle, wsid);
    fpgaClose(accel_handle);

    return 0;
}
