/**
 * Copyright (C) 2016-2019 Xilinx, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You may
 * not use this file except in compliance with the License. A copy of the
 * License is located at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

/**
 * Embedded runtime scheduler
 */

#include "core/include/ert.h"
// includes from bsp
#ifndef ERT_HW_EMU
#include <xil_printf.h>

#include <mb_interface.h>
#include <xparameters.h>
#else
#include <stdio.h>
#define xil_printf printf
#define print printf
#define u32 uint32_t
#endif
#include <stdlib.h>
#include <limits>
#include <bitset>
#include <cstring>

// version is a git hash passed in from build script
// default for builds that bypass build script
#ifndef ERT_VERSION
# define ERT_VERSION 0
#endif
#ifndef ERT_SVERSION
# define ERT_SVERSION "0xdeadbeef"
#endif

// set local string that can be extracted
// from binary using 'strings sched.bin'
const char* ert_v30_version = ERT_SVERSION;

#define ERT_UNUSED __attribute__((unused))

//#define ERT_VERBOSE
#define CTRL_VERBOSE
//#define DEBUG_SLOT_STATE

// Assert macro implementation
ERT_UNUSED
static void
ert_assert(const char* file, long line, const char* function, const char* expr, const char* msg)
{
  xil_printf("Assert failed: %s:%ld:%s:%s %s\r\n",file,line,function,expr,msg);
  exit(1);
}

#ifdef ERT_VERBOSE
# define ERT_PRINT(msg) print(msg)
# define ERT_DEBUG(msg) print(msg)
# define ERT_PRINTF(format,...) xil_printf(format, ##__VA_ARGS__)
# define ERT_DEBUGF(format,...) xil_printf(format, ##__VA_ARGS__)
# define ERT_ASSERT(expr,msg) ((expr) ? ((void)0) : ert_assert(__FILE__,__LINE__,__FUNCTION__,#expr,msg))
#else
# define ERT_PRINT(msg) print(msg)
# define ERT_DEBUG(msg)
# define ERT_PRINTF(format,...) xil_printf(format, ##__VA_ARGS__)
# define ERT_DEBUGF(format,...)
# define ERT_ASSERT(expr,msg)
//# define ERT_ASSERT(expr,msg) ((expr) ? ((void)0) : ert_assert(__FILE__,__LINE__,__FUNCTION__,#expr,msg))
#endif

#ifdef CTRL_VERBOSE
# define CTRL_DEBUG(msg) print(msg)
# define CTRL_DEBUGF(format,...) xil_printf(format, ##__VA_ARGS__)
# define DMSGF(format,...) if (dmsg) xil_printf(format, ##__VA_ARGS__)
#else
# define CTRL_DEBUG(msg)
# define CTRL_DEBUGF(format,...)
# define DMSGF(format,...)
#endif

#ifdef ERT_HW_EMU
using addr_type = uint32_t;
using value_type = uint32_t;

extern value_type read_reg(addr_type addr);
extern void write_reg(addr_type addr, value_type val);
extern void microblaze_enable_interrupts();
extern void microblaze_disable_interrupts();
extern void reg_access_wait();
#endif

namespace ert_v30 {
////////////////////////////////////////////////////////////////
// Convenience types for clarity
////////////////////////////////////////////////////////////////
using size_type = uint32_t;
using addr_type = uint32_t;
using value_type = uint32_t;
using bitmask_type = uint32_t;

////////////////////////////////////////////////////////////////
// HLS AXI-lite
////////////////////////////////////////////////////////////////
static const u32 AP_START    = 0x1;
static const u32 AP_DONE     = 0x2;
static const u32 AP_IDLE     = 0x4;
static const u32 AP_READY    = 0x8;
static const u32 AP_CONTINUE = 0x10;

////////////////////////////////////////////////////////////////
// HLS AXI protocol (from xclbin.h)
////////////////////////////////////////////////////////////////
#define	CU_ADDR_HANDSHAKE_MASK	(0xff)
#define	CU_HANDSHAKE(addr) ((addr) & CU_ADDR_HANDSHAKE_MASK)
#define	CU_ADDR(addr) ((addr) & ~CU_ADDR_HANDSHAKE_MASK)
static const u32 AP_CTRL_HS    = 0;
static const u32 AP_CTRL_CHAIN = 1;
static const u32 AP_CTRL_NONE  = 2;
static const u32 AP_CTRL_ME    = 3;
static const u32 ACCEL_ADATER  = 4;
static const u32 FAST_ADATER   = 5;

////////////////////////////////////////////////////////////////
// Extensions to core/include/ert.h
////////////////////////////////////////////////////////////////
addr_type STATUS_REGISTER_ADDR[4] = {0, 0, 0, 0};

addr_type CU_DMA_REGISTER_ADDR[4] = {0, 0, 0, 0};

addr_type CU_STATUS_REGISTER_ADDR[4] = {0, 0, 0, 0};

addr_type CQ_STATUS_REGISTER_ADDR[4] = {0, 0, 0, 0};

addr_type CU_IPR[4] = {0, 0, 0, 0};

value_type CQ_STATUS[4] = {0, 0, 0, 0};

value_type COMPLETE_SLOT[4] = {0, 0, 0, 0};

value_type CU_PEND_SLOT[128][4] = {0};


/**
 * Simple bitset type supporting 128 bits
 *
 * ERT supports a max of 128 CUs and 128 slots, this bitset class
 * is added to simplify managing 4 32bit bitmasks.
 *
 * Using std::bitset non-throwing functions only
 */
using bitset_type = std::bitset<128>;

// If this assert fails, then ert_parameters is out of sync with
// the board support package header files.
#ifndef ERT_HW_EMU
static_assert(ERT_INTC_ADDR==XPAR_INTC_SINGLE_BASEADDR,"update core/include/ert.h");
#endif

// Marker for invalid index
const size_type no_index = std::numeric_limits<size_type>::max();

////////////////////////////////////////////////////////////////
// Configuarable constants
// Statically allcoated array size is reduced in debug otherwise
// there is not enough space for compiled firmware
////////////////////////////////////////////////////////////////
#ifdef ERT_VERBOSE
const  size_type max_slots                  = 32;   // size of statically allocated array
#else
const  size_type max_slots                  = 128;  // size of statically allocated array
#endif
static size_type num_slots                  = 16;   // actual number of slots
static size_type num_slot_masks             = 1;    // (num_slots-1>>5)+1;

// Max number of compute units
#ifdef ERT_VERBOSE
const  size_type max_cus                    = 32;   // size of statically allocated array
#else
const  size_type max_cus                    = 128;  // size of statically allocated array
#endif
static size_type num_cus                    = 3;    // actual number of cus
static size_type num_cu_masks               = 1;    // (num_cus-1>>5)+1;

// CU base address
static addr_type cu_base_address            = 0x0;

// CU address map
static addr_type cu_addr_map[max_cus]       = {0};

// CU offset (addone is 64k (1<<16), OCL is 4k (1<<12)) 
static size_type cu_offset                  = 16;

// Slot size
static size_type slot_size                  = 0x1000;

// Enable features via  configure_mb
static value_type cq_status_enabled         = 0;
static value_type mb_host_interrupt_enabled = 0;
static value_type dataflow_enabled          = 0;
static value_type kds_30                    = 0;
static value_type dmsg                      = 0;
/* Performance breakdown: if echo flag is set,
 * MB notifies host right away and does not touch
 * hardware(config CU)
 */
static value_type echo                      = 0;

static value_type intr                      = 0;

static value_type polling                   = 1;

// Struct slot_info is per command slot in command queue
struct slot_info
{
  // Address of slot in command queue
  addr_type slot_addr = 0;

  // Last command header read from slot in command queue
  // Last 4 bits of header are used for slot status per mb state
  // new     [0x1]: the command is in new state per host
  // queued  [0x2]: the command is queued in MB.
  // running [0x3]: the command is running
  // free    [0x4]: the command slot is free
  value_type header_value = 0;

  // Cache opcode
  value_type opcode = 0;

  // Index of CU that is assigned to this command
  size_type cu_idx = no_index;

  // Address of register map in command slot
  addr_type regmap_addr = 0;

  // Size of register map in command slot (in 32 bit words)
  size_type regmap_size = 0;
};

// Fixed sized map from slot_idx -> slot info
static slot_info command_slots[max_slots];

// Fixed sized map from cu_idx -> slot_idx
static size_type cu_slot_usage[max_cus];

// Fixed sized map from cu_idx -> number of times executed
static size_type cu_usage[max_cus];

static value_type cq_new[max_slots];

static value_type level1_idx[max_slots];


static size_type start_t = 0;
static size_type end_t = 0;
// Bitmask indicating status of CUs. (0) idle, (1) running.
// Only 'num_cus' lower bits are used
static bitset_type cu_status;

static bitset_type slot_submitted;

//static bitset_type cq_new;

static bitset_type cu_ready;

static bitset_type cu_done;

static bitset_type cu_args_cached;
// Bitmask for interrupt enabled CUs.  (0) no interrupt (1) enabled
static bitset_type cu_interrupt_mask;
#ifndef ERT_HW_EMU
/**
 * Utility to read a 32 bit value from any axi-lite peripheral
 */
inline value_type
read_reg(addr_type addr)
{
  volatile auto ptr = reinterpret_cast<addr_type*>(addr);
  return *ptr;
}

/**
 * Utility to write a 32 bit value from any axi-lite peripheral
 */
inline void
write_reg(addr_type addr, value_type val)
{
  ERT_DEBUGF("write_reg addr(0x%x) val(0x%x)\r\n", addr, val);
  volatile auto ptr = reinterpret_cast<addr_type*>(addr);
  *ptr = val;
}
#endif
/**
 * Command opcode [27:23]
 */
inline value_type
opcode(value_type header_value)
{
  return (header_value >> 23) & 0x1F;
}
  
/**
 * Command type [31:28]
 */
inline value_type
cmd_type(value_type header_value)
{
  return (header_value >> 28) & 0xF;
}

/**
 * Command header [22:12] is payload size
 */
inline size_type
payload_size(value_type header_value)
{
  return (header_value >> 12) & 0x7FF;
}

/**
 * Command header [11:10] is extra cu masks.
 */
inline size_type
cu_masks(value_type header_value)
{
  return 1 + ((header_value >> 10) & 0x3);
}

/**
 * CU section (where the cu bitmasks start)
 */
inline addr_type
cu_section_addr(addr_type slot_addr)
{
  return slot_addr + sizeof(addr_type);
}

/**
 * Regmap section (where the cu regmap) is immediately
 * after cu section
 */
inline addr_type
regmap_section_addr(value_type header_value,addr_type slot_addr)
{
  return cu_section_addr(slot_addr) + cu_masks(header_value)*sizeof(addr_type);
}

/**
 * Size of regmap is payload size (n) minus the number of cu_masks
 */
inline size_type
regmap_size(value_type header_value)
{
  return payload_size(header_value) - cu_masks(header_value);
}

inline addr_type
cu_idx_to_addr(size_type cu_idx)
{
  return CU_ADDR(cu_addr_map[cu_idx]);
}

inline value_type
cu_idx_to_ctrl(size_type cu_idx)
{
  return CU_HANDSHAKE(cu_addr_map[cu_idx]);
}
  
/**
 * idx_in_mask() - Check if idx in in specified 32 bit mask
 *
 * @idx: Index to check
 * @mask_idx: Index of bit mask determines range of mask (1=>[63,32])
 * Return: @true of idx is in range of 32 bit mask, @false otherwise
 */
inline bool
idx_in_mask(size_type idx, size_type mask_idx)
{
  return idx < ((mask_idx+1)<<5);
}

/**
 * idx_to_mask() - Return the bitmask corresponding to idx in mask with idx
 *
 * @idx: Index to map to specified mask
 * @mask_idx: Index of bit mask determines range of mask (1=>[63,32])
 * Return: 32 bit bitmask with position of translated idx set to '1', or 0x0
 *  if idx is not covered by range of mask
 */
inline bitmask_type
idx_to_mask(size_type idx, size_type mask_idx)
{
  return idx_in_mask(idx,mask_idx)
    ? 1 << (idx-(mask_idx<<5))
    : 0;
}

// scope guard for disabling interrupts
struct disable_interrupt_guard
{
  disable_interrupt_guard()
  {
    if (!polling || cq_status_enabled)
      write_reg(ERT_INTC_MER_ADDR,0x0); // interrupt controller master disable
  }
  ~disable_interrupt_guard()
  {
    if (!polling || cq_status_enabled)
      write_reg(ERT_INTC_MER_ADDR,0x3); // interrupt controller master enable
  }
};

inline static void
setup_ert_base_addr()
{
  // Prevent setup() from writing to unitialized bogus addresses
  std::fill(cu_addr_map, cu_addr_map + max_cus, AP_CTRL_NONE);
  
  // In Subsytem 2.0, ERT MB now has to go around to access 3 peripherals
  // internal to ERT Subsystem i.e CQRAM Controller, Embedded scheduler HW
  // and KDMA. ERT MB needs to read the value in ERT_BASE_ADDR, add that
  // value to ERT Subsystem Base Address and Peripheral Address, new value
  // would be used by the ERT MB to access CQ and CSR.
  #if defined(ERT_BUILD_V30)
  ert_base_addr = read_reg(ERT_BASE_ADDR);
  #endif
  STATUS_REGISTER_ADDR[0] = ERT_STATUS_REGISTER_ADDR0;
  STATUS_REGISTER_ADDR[1] = ERT_STATUS_REGISTER_ADDR1;
  STATUS_REGISTER_ADDR[2] = ERT_STATUS_REGISTER_ADDR2;
  STATUS_REGISTER_ADDR[3] = ERT_STATUS_REGISTER_ADDR3;
  CU_DMA_REGISTER_ADDR[0] = ERT_CU_DMA_REGISTER_ADDR0;
  CU_DMA_REGISTER_ADDR[1] = ERT_CU_DMA_REGISTER_ADDR1;
  CU_DMA_REGISTER_ADDR[2] = ERT_CU_DMA_REGISTER_ADDR2;
  CU_DMA_REGISTER_ADDR[3] = ERT_CU_DMA_REGISTER_ADDR3;
  CU_STATUS_REGISTER_ADDR[0] = ERT_CU_STATUS_REGISTER_ADDR0;
  CU_STATUS_REGISTER_ADDR[1] = ERT_CU_STATUS_REGISTER_ADDR1;
  CU_STATUS_REGISTER_ADDR[2] = ERT_CU_STATUS_REGISTER_ADDR2;
  CU_STATUS_REGISTER_ADDR[3] = ERT_CU_STATUS_REGISTER_ADDR3;
  CQ_STATUS_REGISTER_ADDR[0] = ERT_CQ_STATUS_REGISTER_ADDR0;
  CQ_STATUS_REGISTER_ADDR[1] = ERT_CQ_STATUS_REGISTER_ADDR1;
  CQ_STATUS_REGISTER_ADDR[2] = ERT_CQ_STATUS_REGISTER_ADDR2;
  CQ_STATUS_REGISTER_ADDR[3] = ERT_CQ_STATUS_REGISTER_ADDR3;
  CU_IPR[0] = ERT_INTC_CU_0_31_IPR;
  CU_IPR[1] = ERT_INTC_CU_32_63_IPR;
  CU_IPR[2] = ERT_INTC_CU_64_95_IPR;
  CU_IPR[3] = ERT_INTC_CU_96_127_IPR;
}

/**
 * MB configuration
 */
static void
setup()
{
  CTRL_DEBUG("-> v30 setup\r\n");

  // In dataflow number of slots is number of CUs plus ctrl slot (0),
  // otherwise its as many slots as possible per slot_size
  num_slots = ERT_CQ_SIZE / slot_size;
  num_slot_masks = ((num_slots-1)>>5) + 1;
  num_cu_masks = ((num_cus-1)>>5) + 1;

  CTRL_DEBUGF("slot_size=0x%x\r\n",slot_size);
  CTRL_DEBUGF("num_slots=%d\r\n",num_slots);
  CTRL_DEBUGF("num_slot_masks=%d\r\n",num_slot_masks);
  CTRL_DEBUGF("num_cus=%d\r\n",num_cus);
  CTRL_DEBUGF("num_cu_masks=%d\r\n",num_cu_masks);
  CTRL_DEBUGF("cu_offset=%d\r\n",cu_offset);
  CTRL_DEBUGF("cu_base_address=0x%x\r\n",cu_base_address);
  CTRL_DEBUGF("cq_int_enabled=%d\r\n",cq_status_enabled);
  CTRL_DEBUGF("mb_host_int_enabled=%d\r\n",mb_host_interrupt_enabled);
  CTRL_DEBUGF("dataflow_enabled=%d\r\n",dataflow_enabled);
  CTRL_DEBUGF("kds_30=%d\r\n",kds_30);
  CTRL_DEBUGF("dmsg=%d\r\n",dmsg);
  CTRL_DEBUGF("echo=%d\r\n",echo);
  CTRL_DEBUGF("polling=%d\r\n",polling);
  CTRL_DEBUGF("end_t=%d\r\n",end_t);
  CTRL_DEBUGF("start_t=%d\r\n",start_t);
  // Initialize command slots
  for (size_type i=0; i<num_slots; ++i) {
    auto& slot = command_slots[i];
    slot.slot_addr = ERT_CQ_BASE_ADDR + (slot_size * i);
    slot.header_value = 0x4; // free
    slot.cu_idx = no_index;
    slot.regmap_addr = 0;
    slot.regmap_size = 0;

    // Clear command queue headers memory
    write_reg(slot.slot_addr,0x0);

    memset(CU_PEND_SLOT[i], 0, 16);

    cq_new[i] = 0;
  }

  // Clear CSR  (COR so read)
  for (size_type i=0; i<4; ++i) {
    ERT_UNUSED volatile auto val = read_reg(STATUS_REGISTER_ADDR[i]);
    CQ_STATUS[i] = 0;
    COMPLETE_SLOT[i] = 0;
  }

  cu_status.reset();
  cu_ready.reset();
  cu_done.reset();
  slot_submitted.reset();
  cu_args_cached.reset();
  

  // Initialize cu_slot_usage
  for (size_type i=0; i<num_cus; ++i) {
    cu_slot_usage[i] = no_index;
    cu_usage[i] = 0;
  }

  // Set slot size (4K)
  write_reg(ERT_CQ_SLOT_SIZE_ADDR,slot_size/4);

  // CU offset in shift value

  write_reg(ERT_CU_OFFSET_ADDR,cu_offset);

  // Number of command slots
  write_reg(ERT_CQ_NUMBER_OF_SLOTS_ADDR,num_slots);

  // CU base address
  write_reg(ERT_CU_BASE_ADDRESS_ADDR,cu_base_address/4);

  // Command queue base address
  // The HW has changed so that the CQ is accessed at a different
  // address by the cudma_controller (which is internal to the ERT
  // subsystem) than the address it is accessed to my masters which
  // are external to the ERT subsystem e.g.XRT.
  //
  // So the cudma_controller has the CQ BRAM mapped at address
  // 0x0000_0000, but it will be at address ERT_CQ_BASE_ADDR
  // (i.e. 0x0034_0000) for XRT. And the code above was updated to
  // reflect this.
  write_reg(ERT_CQ_BASE_ADDRESS_ADDR,0x0);


  // Number of CUs
  write_reg(ERT_NUMBER_OF_CU_ADDR,num_cus);

  bool enable_master_interrupts = false;

  // Enable cu interupts (cu -> cu_isr -> mb interrupts)
  cu_interrupt_mask.reset();
  bitmask_type intc_ier_mask = 0;

  if (kds_30) {

    for (size_type cu=0; cu<num_cus; ++cu) {
      if (cu_idx_to_ctrl(cu) == AP_CTRL_NONE)
        continue;
      write_reg(cu_idx_to_addr(cu) + 0x4, 1);
      write_reg(cu_idx_to_addr(cu) + 0x8, 1);
      if (intr)
        cu_interrupt_mask[cu] = 1;
    }

    if (intr) {
      intc_ier_mask |= 0x1E0; // acccept cu interrupts on bit 1 of the ier of intc
      enable_master_interrupts = true;
      write_reg(ERT_INTC_CU_0_31_MER,0x3);            // interrupt controller master enable
      write_reg(ERT_INTC_CU_32_63_MER,0x3);           // interrupt controller master enable
      write_reg(ERT_INTC_CU_64_95_MER,0x3);           // interrupt controller master enable
      write_reg(ERT_INTC_CU_96_127_MER,0x3);          // interrupt controller master enable
    }
    write_reg(ERT_INTC_CU_0_31_IER,0xFFFFFFFF);     // accept interrupts for features
    write_reg(ERT_INTC_CU_32_63_IER,0xFFFFFFFF);    // accept interrupts for features
    write_reg(ERT_INTC_CU_64_95_IER,0xFFFFFFFF);    // accept interrupts for features
    write_reg(ERT_INTC_CU_96_127_IER,0xFFFFFFFF);   // accept interrupts for features

  }
  CTRL_DEBUGF("cu interrupt mask : %s\r\n", "cannot convert"); // to_string throws

  // Enable interrupts from host to MB when new commands are ready
  // When enabled, MB will read CQ_STATUS_REGISTER(s) to determine new
  // command slots.
  if (cq_status_enabled) {
    write_reg(ERT_CQ_STATUS_ENABLE_ADDR,1);   // enable feature
    intc_ier_mask |= 0x1;                     // acccept interrupts on bit 0 of the ier of intc
    enable_master_interrupts = true;
  } else {
    write_reg(ERT_INTC_IER_ADDR,read_reg(ERT_INTC_IER_ADDR) & ~0x1);  // disable interrupts on bit 0
    write_reg(ERT_CQ_STATUS_ENABLE_ADDR,0);      // disable feature
  }

  if (enable_master_interrupts) {
    intc_ier_mask |= 0xB;
#if defined(ERT_BUILD_V30)
    CTRL_DEBUGF("Enable and 0x1f2000 IER\r\n");
    write_reg(ERT_INTC_IER_ADDR,intc_ier_mask);  // accept interrupts for features
    write_reg(ERT_INTC_MER_ADDR,0x3);            // interrupt controller master enable
#endif
    microblaze_enable_interrupts();          // enable interrupts
  } else {
    write_reg(ERT_INTC_MER_ADDR,read_reg(ERT_INTC_MER_ADDR) & ~0x3);           // interrupt controller master enable
    microblaze_disable_interrupts();          // disable interrupts
  }

  // Enable/disable mb->host interrupts
  write_reg(ERT_HOST_INTERRUPT_ENABLE_ADDR,mb_host_interrupt_enabled);
  CTRL_DEBUG("<- setup\r\n");
}

/**
 * Associate CUs with a command slot
 */
inline void
set_cu_info(size_type cu_idx, size_type slot_idx)
{
  DMSGF("cu_slot_usage[%d]=%d\r\n",cu_idx,slot_idx);
  ERT_ASSERT(cu_slot_usage[cu_idx]==no_index,"cu already used");
  cu_slot_usage[cu_idx] = slot_idx;
  ++cu_usage[cu_idx];
}

/**
 * write_command_status
 */
inline void
notify_host(size_type cmd_idx)
{
  // notify host (update host status register)
  DMSGF("notify_host(%d)\r\n",cmd_idx);
#if 0
  auto mask_idx = cmd_idx>>5;
  auto mask = idx_to_mask(cmd_idx,mask_idx);
  write_reg(STATUS_REGISTER_ADDR[mask_idx],mask);
#else
  // this relies on 1<<n == 1<<(n%32), safe?  no, not safe, to be fixed
  // once 128 slots work so I can check the actual behavior
  write_reg(STATUS_REGISTER_ADDR[cmd_idx>>5],1<<cmd_idx);
#endif
}

/**
 * Configure a CU at argument address
 *
 * Write register map to CU control register at address
 *
 * @param cu_addr
 *  Address of CU control register
 * @param regmap_addr
 *  The address of the register map to copy into the CU control
 *  register
 * @param regmap_size
 *  The size of register map in 32 bit words
 */
inline void
configure_cu(addr_type cu_addr, addr_type regmap_addr, size_type regmap_size)
{
  uint32_t *addr_ptr = (uint32_t *)(uintptr_t)cu_addr;
  uint32_t *regmap_ptr = (uint32_t *)(uintptr_t)regmap_addr;

  memcpy(addr_ptr+4, regmap_ptr+4, (regmap_size-4)<<2);

  // start kernel at base + 0x0
  write_reg(cu_addr, 0x1);
}

/**
 * Configure CU with address value pairs (out-of-order)
 */
inline void
configure_cu_ooo(addr_type cu_addr, addr_type regmap_addr, size_type regmap_size)
{
  // write register map addr, value pairs starting 
  // past reserved 4 ctrl + 2 ctx 
  for (size_type idx = 6; idx < regmap_size; idx += 2) {
    addr_type offset = read_reg(regmap_addr + (idx << 2));
    value_type value = read_reg(regmap_addr + ((idx + 1) << 2));
    write_reg(cu_addr + offset, value);
  }

  // start kernel at base + 0x0
  write_reg(cu_addr, 0x1);
}

/**
 * Start a CU for command in slot
 *
 * @param slot_idx
 *  Index of command
 * @return
 *  Index of CU that was started or no_index if no CU was
 *  started (all were busy).
 *
 * Command is already assigned a CU by KDS.  This function checks
 * the current ERT status of that CU and starts it if it is unused.
 */
inline size_type
start_cu(size_type slot_idx)
{
  auto& slot = command_slots[slot_idx];
  auto cu_idx = slot.cu_idx;

  if (cu_status[cu_idx])
    return no_index;

  DMSGF("start_cu cu(%d) for slot_idx(%d)\r\n",cu_idx,slot_idx);
  ERT_ASSERT(read_reg(cu_idx_to_addr(cu_idx))==AP_IDLE,"cu not ready");

  if (slot.opcode==ERT_EXEC_WRITE)
    // Out of order configuration
    configure_cu_ooo(cu_idx_to_addr(cu_idx),slot.regmap_addr,slot.regmap_size);
  else
    // manually configure and start cu
    configure_cu(cu_idx_to_addr(cu_idx),slot.regmap_addr,slot.regmap_size);
  
  cu_status[cu_idx] = !cu_status[cu_idx];     // toggle cu status bit, it is now busy
  set_cu_info(cu_idx,slot_idx); // record which slot cu associated with
  return cu_idx;
}

/**
 * Check command status
 *
 * Called from CU interrupt service routine.  Argument CU is
 * complete, notify host and update state to indiciate slot
 * is now free.
 *
 * @param slot_idx
 *  Index of command slot that is associated with the CU
 * @param cu_idx
 *  Index of CU that just completed
 */
static inline void
check_command(size_type slot_idx, size_type cu_idx)
{
  auto& slot = command_slots[slot_idx];
  DMSGF("slot.cu_idx(%d) slot_idx(%d)\r\n",slot.cu_idx,slot_idx);
  ERT_ASSERT(slot.cu_idx == cu_idx,"cu is not used by slot");
  notify_host(slot_idx);
  slot.header_value = (slot.header_value & ~0xF) | 0x4; // free
  DMSGF("slot(%d) [running -> free]\r\n",slot_idx);

#ifdef DEBUG_SLOT_STATE
  write_reg(slot.slot_addr,slot.header_value);
#endif
}

/**
 * Check CU status and update internal state if complete
 &
 * @param cu_idx
 *   Index of CU to check
 * @return
 *   True if CU is done, false otherwise
 */
ERT_UNUSED
static inline bool
check_cu(size_type cu_idx)
{
  // cu interrupt enabled, then managed by interrupt handler
  if (cu_interrupt_mask[cu_idx])
    return false;

  // no interrupt for cu, check cu status register manually
  ERT_ASSERT(cu_status[cu_idx],"cu wasn't started");

  if (!(read_reg(cu_idx_to_addr(cu_idx)) & AP_DONE))
    return false;
  // #### write AP_CONTINUE in data_flow

  // toogle cu status bit, it is now free
  cu_status[cu_idx] = !cu_status[cu_idx];
  cu_slot_usage[cu_idx] = no_index; // reset slot index
  return true;
}

/**
 * Configure MB and peripherals
 *
 * Wait for CONFIGURE_MB in specified slot, then configure as
 * requested.
 *
 * This function is used in two different scenarios:
 *  1. MB reset/startup, in which case the CONFIGURE_MB is guaranteed
 *     to be in a slot at default slot offset (4K), most likely slot 0.
 *  2. During regular scheduler loop, in which case the CONFIGURE_MB
 *     packet is at an arbitrary slot location.   In this scenario, the
 *     function may return (false) without processing the command if
 *     other commands are currently executing; this is to avoid hardware
 *     lockup.
 *
 * @param slot_idx
 *   The slot index with the CONFIGURE_MB command
 * @return
 *   True if CONFIGURE_MB packet was processed, false otherwise
 */
static bool
configure_mb(size_type slot_idx)
{
  CTRL_DEBUGF("-->configure_mb\r\n");
  auto& slot = command_slots[slot_idx];

  CTRL_DEBUGF("configure cmd found in slot(%d)\r\n",slot_idx);
  slot_size=read_reg(slot.slot_addr + 0x4);
  num_cus=read_reg(slot.slot_addr + 0x8);
  cu_offset=read_reg(slot.slot_addr + 0xC);
  cu_base_address=read_reg(slot.slot_addr + 0x10);

  // Features
  auto features = read_reg(slot.slot_addr + 0x14);
  CTRL_DEBUGF("features=0x%04x\r\n",features);
  ERT_ASSERT(features & 0x1,"ert is not enabled!!");
  mb_host_interrupt_enabled = (features & 0x2)==0;

  cq_status_enabled = (features & 0x10)!=0;
  dataflow_enabled = (features & 0x40)!=0;
  kds_30 = (features & 0x100)!=0;
  dmsg = (features & 0x200)!=0;
  echo = (features & 0x400)!=0;
  intr = (features & 0x800)!=0;
  polling = (!intr && (dataflow_enabled || kds_30));

  // CU base address
  for (size_type i=0; i<num_cus; ++i) {
    u32 addr = read_reg(slot.slot_addr + 0x18 + (i<<2));
    cu_addr_map[i] = addr;  // encoded with handshake
    CTRL_DEBUGF("cu(%d) addr(0x%x) handshake(0x%x) encodedaddr(0x%x)\r\n", i, CU_ADDR(addr), CU_HANDSHAKE(addr), addr);
  }

  // (Re)initilize MB
  setup();

  // notify host
  notify_host(slot_idx);

  slot.header_value = (slot.header_value & ~0xF) | 0x4; // free
  CTRL_DEBUGF("slot(%d) [running -> free]\r\n",slot_idx);
  CTRL_DEBUGF("before CQ_STATUS[0] %d\r\n", CQ_STATUS[slot_idx>>5]);
  CQ_STATUS[slot_idx>>5] &= ~(1<<(slot_idx&0xFFFFFFFF));
  CTRL_DEBUGF("after CQ_STATUS[0] %d\r\n", CQ_STATUS[slot_idx>>5]);
  CTRL_DEBUGF("<--configure_mb\r\n");
  return true;
}

static bool
exit_mb(size_type slot_idx)
{
  auto& slot = command_slots[slot_idx];
  CTRL_DEBUGF("exit_mb slot(%d) header=0x%x\r\n",slot_idx,slot.header_value);

  if (kds_30 && intr) {
    write_reg(ERT_INTC_CU_0_31_MER,0);            // interrupt controller master disable
    write_reg(ERT_INTC_CU_32_63_MER,0);           // interrupt controller master disable
    write_reg(ERT_INTC_CU_64_95_MER,0);           // interrupt controller master disable
    write_reg(ERT_INTC_CU_96_127_MER,0);            // interrupt controller master disable    
  }

  // Update registers so mgmt driver knows ERT has exited
  slot.header_value = (slot.header_value & ~0xF) | 0x4; // free
  write_reg(slot.slot_addr,slot.header_value); // acknowledge the completed control command
  CTRL_DEBUGF("scheduler loop exits slot(%d) header=0x%x\r\n",slot_idx,slot.header_value);
#ifndef ERT_HW_EMU
  mb_sleep();
#endif
  return true;
}

// Gather ERT stats in ctrl command packet
// [1  ]      : header
// [1  ]      : custat version
// [1  ]      : ert version
// [1  ]      : number of cq slots
// [1  ]      : number of cus
// [#numcus]  : cu execution stats (number of executions)
// [#numcus]  : cu status (1: running, 0: idle)
// [#slots]   : command queue slot status
static bool
cu_stat(size_type slot_idx)
{
  auto& slot = command_slots[slot_idx];
  CTRL_DEBUGF("slot(%d) [new -> queued -> running]\r\n",slot_idx);
  CTRL_DEBUGF("cu_stat slot(%d) header=0x%x\r\n",slot_idx,slot.header_value);

  // write stats to command package after header
  size_type pkt_idx = 1; // after header
  size_type max_idx = slot_size >> 2; 

  // custat version, update when changing layout of packet
  write_reg(slot.slot_addr + (pkt_idx++ << 2),0x51a10000);

  // ert git version
  write_reg(slot.slot_addr + (pkt_idx++ << 2),ERT_VERSION);

  // number of cq slots
  write_reg(slot.slot_addr + (pkt_idx++ << 2),num_slots);

  // number of cus
  write_reg(slot.slot_addr + (pkt_idx++ << 2),num_cus);
  
  // individual cu execution stat
  for (size_type i=0; pkt_idx<max_idx && i<num_cus; ++i) {
    DMSGF("cu_usage[0x%x]=%d\r\n",cu_idx_to_addr(i),cu_usage[i]);
    write_reg(slot.slot_addr + (pkt_idx++ << 2),cu_usage[i]);
  }

  // individual cu status
  for (size_type i=0; pkt_idx<max_idx && i<num_cus; ++i) {
    DMSGF("cu_staus[0x%x]=%d\r\n",cu_idx_to_addr(i),(uint32_t)cu_status[i]);
    write_reg(slot.slot_addr + (pkt_idx++ << 2),cu_status[i]);
  }

  // command slot status
  for (size_type i=0; pkt_idx<max_idx && i<num_slots; ++i) {
    auto& s = command_slots[i];
    DMSGF("slot_status[%d]=%d\r\n",i,s.header_value & 0XF);
    write_reg(slot.slot_addr + (pkt_idx++ << 2),s.header_value & 0XF);
  }

#if 0
  // payload count
  auto mask = 0X7FF << 12;  // [22-12]
  slot.header_value = (slot.header_value & (~mask)) | (pkt_idx << 12);
  CTRL_DEBUGF("cu_stat new header=0x%x\r\n",slot.header_value);
  write_reg(slot.slot_addr, slot.header_value);
#endif

  // notify host
  notify_host(slot_idx);
  slot.header_value = (slot.header_value & ~0xF) | 0x4; // free
  CTRL_DEBUGF("slot(%d) [running -> free]\r\n",slot_idx);
  return true;
}

static bool
abort_mb(size_type slot_idx)
{
  CTRL_DEBUGF("abort cmd found in slot(%d)\r\n",slot_idx);

  disable_interrupt_guard guard;
  auto& slot = command_slots[slot_idx];
  size_type sidx = (slot.header_value >>  15) & 0xFF;
  auto& s = command_slots[sidx];
  if (opcode(s.header_value)!=ERT_START_KERNEL)
    return true; // bail if not a start_kernel command
  if ((s.header_value & 0xF)!=0x3)
    return true; // bail if not running
  auto cu_idx = s.cu_idx;
  check_command(sidx,cu_idx);
  cu_slot_usage[cu_idx] = no_index;
  cu_status[cu_idx] = !cu_status[cu_idx];
  notify_host(slot_idx);
  return true;
}

static bool
counter_calib(size_type slot_idx)
{
  //CTRL_DEBUGF("counter_calib cmd found in slot(%d)\r\n",slot_idx);

  auto& slot = command_slots[slot_idx];
  volatile size_type start, end;
  size_type cnt = 100000000;
  volatile size_type dummy=0;
  slot.opcode = opcode(slot.header_value);

  start = read_reg(0x1F70000);

  while(cnt--){
    dummy++;
  }

  end = read_reg(0x1F70000);

  //CTRL_DEBUGF(" total %d %d dummy %d\r\n",end-start, slot.opcode, dummy);

  write_reg(slot.slot_addr+0x100, end-start);

  notify_host(slot_idx);
  slot.header_value = (slot.header_value & ~0xF) | 0x4;
  return true;
}

/**
 * Process special command.
 *
 * Special commands are not performace critical
 *
 * @return true
 *   If command was processed, false otherwise
 */
static bool
process_special_command(value_type opcode, size_type slot_idx)
{
  if (opcode==ERT_CONFIGURE) // CONFIGURE_MB
    return configure_mb(slot_idx);
  if (opcode==ERT_CU_STAT)
    return cu_stat(slot_idx);
  if (opcode==ERT_EXIT)
    return exit_mb(slot_idx);
  if (opcode==ERT_ABORT)
    return abort_mb(slot_idx);
  if (opcode==ERT_CNT_CALIB)
    return counter_calib(slot_idx);
  return false;
}

/**
 * Transition slot from free to new if possible
 *
 * @return
 *  True if transitioned, false otherwise
 */
inline bool
free_to_new(size_type slot_idx)
{
  // The slot is free and should be updated from command queue
  // only update slot header if host has written 0x1 in header
  auto& slot = command_slots[slot_idx];
  ERT_ASSERT((slot.header_value & 0xF)==0x4,"slot is not free\r\n");
  auto header =  read_reg(slot.slot_addr);
  if ((header & 0xF) == 0x1) {
    if (echo && slot_idx > 0) {
      notify_host(slot_idx);
      slot.header_value = (slot.header_value & ~0xF) | 0x4; // free
      return true;
    }

    DMSGF("new slot(%d)\r\n",slot_idx);
    write_reg(slot.slot_addr,header | 0xF);
    slot.header_value = header;
    DMSGF("slot(%d) [free -> new]\r\n",slot_idx);
    return true;
  }
  return false;
}

/**
 * Transition slot from new to queued
 *
 * @return
 *   True if command was transitioned, false otherwise.
 */
inline bool
new_to_queued(size_type slot_idx)
{
  auto& slot = command_slots[slot_idx];
  ERT_ASSERT((slot.header_value & 0xF)==0x1,"slot is not new\r\n");

  auto cmt = cmd_type(slot.header_value);
  auto opc = slot.opcode = opcode(slot.header_value);
  DMSGF("slot_idx(%d) type(%d) opcode(%d)\r\n",slot_idx,cmt,opc);

  if (cmt != ERT_CU) {
    process_special_command(opc,slot_idx);
    return false;
  }

  // new command, gather slot info
  addr_type addr = cu_section_addr(slot.slot_addr);
  slot.cu_idx = read_reg(addr);
  DMSGF("slot.cu_idx(%d)\r\n",slot.cu_idx);
  slot.regmap_addr = regmap_section_addr(slot.header_value,slot.slot_addr);
  slot.regmap_size = regmap_size(slot.header_value);
  slot.header_value = (slot.header_value & ~0xF) | 0x2; // queued

  DMSGF("slot(%d) [new -> queued]\r\n",slot_idx);

#ifdef DEBUG_SLOT_STATE
  write_reg(slot.slot_addr,slot.header_value);
#endif

  return true;
}

/**
 * Transition slot from queued to running
 *
 * @return
 *   True if command was transitioned, false otherwise.
 */
inline bool
queued_to_running(size_type slot_idx)
{
  auto& slot = command_slots[slot_idx];
  ERT_ASSERT((slot.header_value & 0xF)==0x2,"slot is not queued\r\n");

  // disable CU interrupts while starting command
  disable_interrupt_guard guard;
  // queued command, start if cu is ready
  if (start_cu(slot_idx) == no_index)
    return false;

  slot.header_value |= 0x1;       // running (0x2->0x3)
  DMSGF("slot(%d) [queued -> running]\r\n",slot_idx);

#ifdef DEBUG_SLOT_STATE
  write_reg(slot.slot_addr,slot.header_value);
#endif
  return true;
}

/**
 * Transition slot from running to free
 *
 * @return
 *   True if command was transitioned, false otherwise.
 */
inline bool
running_to_free(size_type slot_idx)
{
  auto& slot = command_slots[slot_idx];
  ERT_ASSERT((slot.header_value & 0xF)==0x3,"slot is not running\r\n");
  
  // running command, check its cu status
  if (!check_cu(slot.cu_idx))
    return false;

  notify_host(slot_idx);
  slot.header_value = (slot.header_value & ~0xF) | 0x4; // free
  DMSGF("slot(%d) [running -> free]\r\n",slot_idx);

#ifdef DEBUG_SLOT_STATE
  write_reg(slot.slot_addr,slot.header_value);
#endif
  return true;
}

inline void cu_hls_ctrl_check(size_type cmd_idx)
{
    auto cuvalue = read_reg(cu_idx_to_addr(cmd_idx));
    DMSGF("cu(%d) is interrupting\r\n",cmd_idx);
    ERT_ASSERT(cu_status[cmd_idx],"cu wasn't started");
    // check if command is done
    check_command(cu_slot_usage[cmd_idx],cmd_idx);
    cu_slot_usage[cmd_idx] = no_index; // reset slot index
    cu_status[cmd_idx] = !cu_status[cmd_idx]; // toggle status of completed cus
    slot_submitted[cmd_idx] = !slot_submitted[cmd_idx];
     
    //CQ_STATUS[cq_sta_idx] &= ~(1<<idx);

    if (cuvalue & (AP_DONE)) {
      DMSGF("AP_DONE \r\n");
      cu_done[cmd_idx] = 1;
      write_reg(cu_idx_to_addr(cmd_idx), AP_CONTINUE);
      write_reg(cu_idx_to_addr(cmd_idx)+0xC, 0x1);
    }

    if (cuvalue & (AP_READY)) {
      DMSGF("AP_READY \r\n");
      cu_ready[cmd_idx] = 1;
    }
}

inline void command_queue_process(void)
{
  for (size_type i=0,offset=0; i<num_slot_masks; ++i,offset+=32) {
    auto slot_mask = read_reg(CQ_STATUS_REGISTER_ADDR[i]);
    //DMSGF("command queue status: 0x%x\r\n",slot_mask);

    for (size_type slot_idx=offset; slot_mask; slot_mask >>= 1, ++slot_idx) {
      //DMSGF("found slot: %d\r\n",slot_idx);
      auto& slot = command_slots[slot_idx];

      if (slot_idx > 0) {
        value_type slot_addr = slot.slot_addr;
        auto val = read_reg(slot_addr);
        if (val & AP_START) {
          write_reg(slot_addr,0x0); // clear
          if (echo) {
            // clear command queue
            notify_host(slot_idx);
            continue;
          }
          //cq_new[slot_idx] = val;
          addr_type addr = cu_section_addr(slot_addr);
          slot.cu_idx = read_reg(addr);

          slot.header_value = val;
          slot.regmap_addr = regmap_section_addr(val,slot_addr);
          slot.regmap_size = regmap_size(val);
          CU_PEND_SLOT[slot.cu_idx][i] |= (1 << (slot_idx%32));
          //DMSGF("CU_PEND_SLOT[%d][%d] = %x\r\n",slot.cu_idx, w, CU_PEND_SLOT[slot.cu_idx][w]);
          level1_idx[slot.cu_idx] |= 1<<i;
        }
        continue;
      }

      if (!cq_status_enabled && ((slot.header_value & 0xF) == 0x4)) { // free
        if (!free_to_new(slot_idx))
          continue;
      }

      if ((slot.header_value & 0xF) == 0x1) { // new
        if (!new_to_queued(slot_idx))
          continue;
      }
    }
  }
}

inline void compute_unit_completion_check(void)
{
  for (size_type i=0, cu_offset=0; i<num_cu_masks; ++i, cu_offset+=32) {
    value_type cu_mask = read_reg(CU_IPR[i]), bck_cu_mask = cu_mask;
    //DMSGF("cu_mask[%d] = %x \r\n", i, cu_mask);
    if (num_cus == 1) // if num_cus = 1, CU_IPR[0] always equal to 0x2 while complete
      cu_mask >>= 1;
    for (size_type cu_idx=cu_offset; cu_mask && cu_idx<num_cus; cu_mask >>= 1, ++cu_idx) {
      if (cu_mask & 0x1) {
        auto cu_slot = cu_slot_usage[cu_idx];

        //DMSGF("cu[%d] done  cu_slot %d\r\n", cu_idx, cu_slot);

        write_reg(cu_idx_to_addr(cu_idx), AP_CONTINUE);
        write_reg(cu_idx_to_addr(cu_idx)+0xC, 0x1); // toggle INTC bit
        //CTRL_DEBUGF(" cu done slot %d, current slot %d\r\n",cu_slot, slot_idx);
        #if 1
        notify_host(cu_slot);
        #else
        COMPLETE_SLOT[cu_slot>>5] |= (1<<(cu_slot));
        #endif
        cu_status[cu_idx] = !cu_status[cu_idx];
      }
    }

    if (bck_cu_mask) {
      //DMSGF("acknowleged INTC mask. num_cus %d\r\n",num_cus);
      write_reg(ERT_INTC_CU_0_31_IAR,bck_cu_mask);
    }
  }
}

inline void compute_unit_start(void)
{
  for (size_type cu_idx=0; cu_idx<num_cus; ++cu_idx) {
    value_type mask = level1_idx[cu_idx];

    if (!mask)
      continue;
    if (cu_status[cu_idx])
      break;

    for (size_type i=0,offset=0; i<num_slot_masks; ++i,offset+=32) {
      value_type pending_slot = CU_PEND_SLOT[cu_idx][i];
      //DMSGF("CU_PEND_SLOT[%d][%d]: %x\r\n",cu_idx, w, pending_slot);
      if (cu_status[cu_idx])
        break;
      if (!pending_slot) {
        mask &= ~(1<<i);
        continue;
      }
      for (size_type slot_idx=offset; pending_slot; pending_slot>>=1, ++slot_idx) {
        auto& slot = command_slots[slot_idx];
        
        if (cu_status[cu_idx])
          break;

        if (!(pending_slot & 0x1))
          continue;

        //DMSGF("kick start cu %d, slot %d\r\n",cu_idx, slot_idx);
        #if 1
        if (slot.opcode==ERT_EXEC_WRITE) // Out of order configuration
          configure_cu_ooo(cu_idx_to_addr(cu_idx),slot.regmap_addr,slot.regmap_size);
        else {
          //value_type start_t, end_t;

          //start_t = read_reg(0x1F70000);
          configure_cu(cu_idx_to_addr(cu_idx),slot.regmap_addr,slot.regmap_size);
          //end_t = read_reg(0x1F70000);
          //CTRL_DEBUGF("time (%d)\r\n", end_t-start_t);
        }
        //}
        cu_status[cu_idx] = !cu_status[cu_idx];
        CU_PEND_SLOT[cu_idx][i] &= ~(1<<slot_idx);
        set_cu_info(cu_idx,slot_idx); // record which slot cu associated with
        #else
        start_cu(slot_idx);
        #endif
        break;
      }
    }
  }
}
/**
 * Main routine executed by embedded scheduler loop
 *
 * For each command slot do
 *  1. If status is free (0x4), then read new command header
 *     Status remains free (0x4), or transitions to new (0x1)
 *  2. If status is new (0x1), then read CUs in command
 *     Status transitions to queued (0x2)
 *  3. If status is queued (0x2), then start command on available CU
 *     Status remains queued if no CUs available, or transitions to running (0x3)
 *  4. If status is running (0x4), then check CU status
 *     Status remains running (0x4) if CU is still running, or
 *     transitions to free if CU is done
 */
static void
scheduler_v30_loop()
{
  ERT_DEBUG("ERT scheduler\r\n");

  // Set up ERT base address, this should only call once
  setup_ert_base_addr();

  // Basic setup will be changed by configure_mb, but is necessary
  // for even configure_mb() to work.
  setup();
  //value_type start_t, end_t;
  while (1) {
#if 1
    //if (kds_30) {
      //CTRL_DEBUGF("kds_30 new flow \r\n");
      // sync CQ
      //start_t = read_reg(0x1F70000);
      command_queue_process();
      //end_t = read_reg(0x1F70000);
      //CTRL_DEBUGF("time (%d)\r\n", end_t-start_t);
      // check CU done
      compute_unit_completion_check();
      // start CU
      compute_unit_start();
      //continue;
    //}
#endif
#if 0
    for (size_type slot_idx=0; slot_idx<num_slots; ++slot_idx) {
      auto& slot = command_slots[slot_idx];
#ifdef ERT_HW_EMU
      reg_access_wait();
#endif
      if (kds_30)
        break;
      if (polling && slot_idx>0 && kds_30) {
#if 0
        auto i = slot_idx>>5;
        value_type complete = COMPLETE_SLOT[i];
        if (complete) {
          write_reg(STATUS_REGISTER_ADDR[i],complete);
          COMPLETE_SLOT[i] = 0;
        }
#endif
        if (!cq_new[slot_idx]) {
          value_type slot_addr = slot.slot_addr;
          auto val = read_reg(slot_addr);
          if (val & AP_START) {
            write_reg(slot.slot_addr,0x0); // clear
            if (echo) {
              // clear command queue
              notify_host(slot_idx);
              continue;
            }

            cq_new[slot_idx] = val;
            addr_type addr = cu_section_addr(slot_addr);
            slot.cu_idx = read_reg(addr);
            slot.header_value = cq_new[slot_idx];
            slot.regmap_addr = regmap_section_addr(slot.header_value,slot_addr);
            slot.regmap_size = regmap_size(slot.header_value);
          }
        }
        // we have nothing else to do
        if (!cq_new[slot_idx])
          continue;

        // there must be some value in cq_new
        // cached cu arguments

        value_type cu_idx = slot.cu_idx;
        // check CU if done

        auto cuvalue = read_reg(cu_idx_to_addr(cu_idx));
        if (cu_status[cu_idx]) {
          //CTRL_DEBUGF(" cuvalue %x\r\n",cuvalue);
          if (cuvalue & (AP_DONE)) { // 153 cycles 
            auto cu_slot = cu_slot_usage[cu_idx];

            write_reg(cu_idx_to_addr(cu_idx), AP_CONTINUE);
            //CTRL_DEBUGF(" cu done slot %d, current slot %d\r\n",cu_slot, slot_idx);
            #if 1
            notify_host(cu_slot);
            #else
            COMPLETE_SLOT[cu_slot>>5] |= (1<<(cu_slot));
            #endif
            cu_status[cu_idx] = !cu_status[cu_idx];
            //CTRL_DEBUGF("cu_status[%d] = (%d)\r\n",cu_idx, (uint32_t)cu_status[cu_idx]);
            cq_new[cu_slot] = 0;
            //CTRL_DEBUGF("cq_new[%d] = (%d)\r\n",cu_slot_usage[cu_idx],cq_new[cu_slot]);
          }
        }

        if (!cu_status[cu_idx]) {
          if (cq_new[slot_idx] & (AP_START)) {
            write_reg(cu_idx_to_addr(cu_idx), AP_CONTINUE);
            //write_reg(cu_idx_to_addr(cu_idx)+0xC, 0x1);

            if (slot.opcode==ERT_EXEC_WRITE) // Out of order configuration
              configure_cu_ooo(cu_idx_to_addr(slot.cu_idx),slot.regmap_addr,slot.regmap_size);
            else {
              value_type start_t, end_t;

              start_t = read_reg(0x1F70000);
              configure_cu(cu_idx_to_addr(cu_idx),slot.regmap_addr,slot.regmap_size);
              end_t = read_reg(0x1F70000);
              CTRL_DEBUGF("time (%d)\r\n", end_t-start_t);
            }
            //}
            cu_status[cu_idx] = !cu_status[cu_idx];
            //cq_new[cu_slot_usage[slot.cu_idx]] = 0;
            //cq_new[slot_idx] = 0;
            set_cu_info(cu_idx,slot_idx); // record which slot cu associated with
            //CTRL_DEBUGF(" cu %d got by slot %d\r\n",slot.cu_idx, slot_idx);
            // CTRL_DEBUGF("cu_slot_usage[%d] = (%d)\r\n",slot.cu_idx,cu_slot_usage[slot.cu_idx]);
          }
        }
        
        continue;
      }
#endif
#if 0
      // In dataflow mode ERT is polling CUs for completion after
      // host has started CU or acknowleged completion.  Ctrl cmds
      // are processed in normal flow.
      if (polling && slot_idx>0 && !kds_30) {

        size_type cuidx = slot_idx-1;  // compensate for reserved slot (0)
        // Check if host has started or continued this CU
        if (!cu_status[cuidx]) {
          auto cqvalue = read_reg(slot.slot_addr);

          if (cqvalue & (AP_START|AP_CONTINUE)) {
            //start_t = read_reg(0x1F70000);
            write_reg(slot.slot_addr,0x0); // clear
            notify_host(slot_idx);
            continue;       
            DMSGF("slot.slot_addr 0x%x enable cu(%d) cqvalue(0x%x)\r\n", slot.slot_addr,cuidx,cqvalue);
            cu_status[cuidx] = !cu_status[cuidx]; // enable polling of this CU
          }
        }

        if (!cu_status[cuidx])
          continue; // CU is not used

        auto cuvalue = read_reg(cu_idx_to_addr(cuidx));
        DMSGF("cuidx %d, cuvalue(0x%x)\r\n",cuidx,cuvalue);
        if (!(cuvalue & (AP_DONE|AP_IDLE)))
          continue;

        cu_status[cuidx] = !cu_status[cuidx]; // disable polling until host re-enables
        // wake up host
        notify_host(slot_idx);
        //end_t = read_reg(0x1F70000);
        //CTRL_DEBUGF("total_cycle=0x%x\r\n",end_t-start_t);
        continue;
      }

      if (!cq_status_enabled && ((slot.header_value & 0xF) == 0x4)) { // free
        if (!free_to_new(slot_idx))
          continue;
      }

      if ((slot.header_value & 0xF) == 0x1) { // new
        if (!new_to_queued(slot_idx))
          continue;
      }

      if ((slot.header_value & 0xF) == 0x2) { // queued
        if (!queued_to_running(slot_idx))
          continue;
      }

      if (!kds_30 && ((slot.header_value & 0xF) == 0x3)) { // running
        if (!running_to_free(slot_idx))
          continue;
      }
    }
    //end_t = read_reg(0x1F70000);
    //CTRL_DEBUGF("total_cycle=0x%x\r\n",end_t-start_t);
    #endif
  } // while
}

/*
static inline void cu_hls_ctrl_check(size_type cmd_idx)
{
    auto cuvalue = read_reg(cu_idx_to_addr(cmd_idx));
    DMSGF("cu(%d) is interrupting\r\n",cmd_idx);
    ERT_ASSERT(cu_status[cmd_idx],"cu wasn't started");
    // check if command is done
    check_command(cu_slot_usage[cmd_idx],cmd_idx);
    cu_slot_usage[cmd_idx] = no_index; // reset slot index
    cu_status[cmd_idx] = !cu_status[cmd_idx]; // toggle status of completed cus

    if (cuvalue & (AP_DONE)) {
      DMSGF("AP_DONE \r\n");
      cu_done[cmd_idx] = 1;
      write_reg(cu_idx_to_addr(cmd_idx), AP_CONTINUE);
      write_reg(cu_idx_to_addr(cmd_idx)+0xC, 0x1);
    }

    if (cuvalue & (AP_READY)) {
      DMSGF("AP_READY \r\n");
      cu_ready[cmd_idx] = 1;
    }
}
*/
/**
 * CU interrupt service routine
 */
#ifndef ERT_HW_EMU
void cu_interrupt_handler() __attribute__((interrupt_handler));
#endif
void
cu_interrupt_handler()
{
  DMSGF("interrupt_handler\r\n");
  bitmask_type intc_mask = read_reg(ERT_INTC_IPR_ADDR);

  if (intc_mask & 0x1) { // host interrupt
    for (size_type w=0,offset=0; w<num_slot_masks; ++w,offset+=32) {
      auto slot_mask = read_reg(CQ_STATUS_REGISTER_ADDR[w]);
      DMSGF("command queue interrupt from host: 0x%x\r\n",slot_mask);
      // Transition each new command into new state
      for (size_type slot_idx=offset; slot_mask; slot_mask >>= 1, ++slot_idx)
        if (slot_mask & 0x1)
          free_to_new(slot_idx);
    }
  }

  for (size_type intc_bit=0x20; intc_bit<0x200; intc_bit <<= 1) {

    if (intc_mask & intc_bit) { // INTC_CU_0_31 ~ INTC_CU_96_127
      DMSGF("intc_mask & 0x%x \r\n",intc_bit);

      bitmask_type cu_intc_mask = 0;
      size_type cu_offset = 0;

      if (intc_bit == 0x20)
        cu_intc_mask = read_reg(ERT_INTC_CU_0_31_IPR);
      else if (intc_bit == 0x40) {
        cu_intc_mask = read_reg(ERT_INTC_CU_32_63_IPR);
        cu_offset = 32;
      }
      else if (intc_bit == 0x80) {
        cu_intc_mask = read_reg(ERT_INTC_CU_64_95_IPR);
        cu_offset = 64;
      }
      else if (intc_bit == 0x100) {
        cu_intc_mask = read_reg(ERT_INTC_CU_96_127_IPR);
        cu_offset = 96;
      }

      DMSGF("cu_intc_mask 0x%x \r\n", cu_intc_mask);
      if (num_cus == 1 && intc_bit == 0x20) {// CU_0 check bit1
        if (0x2 & cu_intc_mask)
          cu_hls_ctrl_check(0);
      } else {
        for (size_type cu_idx=0; cu_idx<32; ++cu_idx) {
          if (1<<cu_idx & cu_intc_mask)
            cu_hls_ctrl_check(cu_offset+cu_idx);
        }
      }
      if (intc_bit == 0x20)
        write_reg(ERT_INTC_CU_0_31_IAR,cu_intc_mask);
      else if (intc_bit == 0x40)
        write_reg(ERT_INTC_CU_32_63_IAR,cu_intc_mask);
      else if (intc_bit == 0x80)
        write_reg(ERT_INTC_CU_64_95_IAR,cu_intc_mask);
      else if (intc_bit == 0x100)
        write_reg(ERT_INTC_CU_96_127_IAR,cu_intc_mask);
    }
  }
  // Acknowledge interrupts
  write_reg(ERT_INTC_IAR_ADDR,intc_mask);
}

} // ert
#ifdef ERT_HW_EMU
#ifdef __cplusplus
extern "C" {
#endif
void scheduler_v30_loop() {
    ert_v30::scheduler_v30_loop();
}

void cu_interrupt_handler_v30() {
    ert_v30::cu_interrupt_handler();
}

#ifdef __cplusplus
}
#endif
#endif

#ifndef ERT_HW_EMU
int main()
{
  ert_v30::scheduler_v30_loop();
  return 0;
}
#endif
