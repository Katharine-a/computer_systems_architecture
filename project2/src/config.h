// Copyright 2025 Blaise Tine
//
// Licensed under the Apache License;
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#define XLEN 32

#define NUM_FUS 3

#define BTB_SIZE  256
#define BHR_SIZE  8

#define ALU_LATENCY 2
#define LSU_LATENCY 100
#define CSR_LATENCY 3

#define CDB_LATENCY 2

#define NUM_RSS 8

#define ROB_SIZE 16

#define NUM_REGS 32

#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 3
#endif

#ifndef RAM_PAGE_SIZE
#define RAM_PAGE_SIZE 4096
#endif

#ifndef MEM_CYCLE_RATIO
#define MEM_CYCLE_RATIO -1
#endif

#ifndef MEMORY_BANKS
#define MEMORY_BANKS 2
#endif

#ifndef MEM_BLOCK_SIZE
#define MEM_BLOCK_SIZE 64
#endif

#ifndef MEM_ADDR_WIDTH
#ifdef XLEN_64
#define MEM_ADDR_WIDTH 48
#else
#define MEM_ADDR_WIDTH 32
#endif
#endif

#ifndef STARTUP_ADDR
#define STARTUP_ADDR 0x80000000
#endif

#ifndef STACK_BASE_ADDR
#define STACK_BASE_ADDR 0xFF000000
#endif

#ifndef IO_BASE_ADDR
#define IO_BASE_ADDR STACK_BASE_ADDR
#endif

#ifndef IO_COUT_ADDR
#define IO_COUT_ADDR IO_BASE_ADDR
#endif
#define IO_COUT_SIZE MEM_BLOCK_SIZE

// Standard CSRs //////////////////////////////////////////////////////////////

#define VX_CSR_SATP                     0x180

#define VX_CSR_PMPCFG0                  0x3A0
#define VX_CSR_PMPADDR0                 0x3B0

#define VX_CSR_MSTATUS                  0x300
#define VX_CSR_MISA                     0x301
#define VX_CSR_MEDELEG                  0x302
#define VX_CSR_MIDELEG                  0x303
#define VX_CSR_MIE                      0x304
#define VX_CSR_MTVEC                    0x305

#define VX_CSR_MEPC                     0x341

#define VX_CSR_MNSTATUS                 0x744

#define VX_CSR_MPM_BASE                 0xB00
#define VX_CSR_MPM_BASE_H               0xB80
#define VX_CSR_MPM_USER                 0xB03
#define VX_CSR_MPM_USER_H               0xB83

#define VX_CSR_MCYCLE                   0xB00
#define VX_CSR_MCYCLE_H                 0xB80
#define VX_CSR_MPM_RESERVED             0xB01
#define VX_CSR_MPM_RESERVED_H           0xB81
#define VX_CSR_MINSTRET                 0xB02
#define VX_CSR_MINSTRET_H               0xB82

#define VX_CSR_MVENDORID                0xF11
#define VX_CSR_MARCHID                  0xF12
#define VX_CSR_MIMPID                   0xF13
#define VX_CSR_MHARTID                  0xF14
