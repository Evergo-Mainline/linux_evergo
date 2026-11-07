// SPDX-License-Identifier: GPL-2.0-only
/*
 * Based on arch/arm/kernel/setup.c
 *
 * Copyright (C) 1995-2001 Russell King
 * Copyright (C) 2012 ARM Ltd.
 */

#include <linux/acpi.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/initrd.h>
#include <linux/console.h>
#include <linux/cache.h>
#include <linux/screen_info.h>
#include <linux/init.h>
#include <linux/kexec.h>
#include <linux/root_dev.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/smp.h>
#include <linux/fs.h>
#include <linux/panic_notifier.h>
#include <linux/proc_fs.h>
#include <linux/memblock.h>
#include <linux/of_fdt.h>
#include <linux/of.h>
#include <linux/firmware.h>
#include <linux/efi.h>
#include <linux/psci.h>
#include <linux/sched/task.h>
#include <linux/scs.h>
#include <linux/mm.h>

#include <asm/acpi.h>
#include <asm/fixmap.h>
#include <asm/cpu.h>
#include <asm/cputype.h>
#include <asm/daifflags.h>
#include <asm/elf.h>
#include <asm/cpufeature.h>
#include <asm/cpu_ops.h>
#include <asm/kasan.h>
#include <asm/numa.h>
#include <asm/rsi.h>
#include <asm/scs.h>
#include <asm/sections.h>
#include <asm/setup.h>
#include <asm/smp_plat.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/traps.h>
#include <asm/efi.h>
#include <asm/xen/hypervisor.h>
#include <asm/mmu_context.h>

static int num_standard_resources;
static struct resource *standard_resources;

phys_addr_t __fdt_pointer __initdata;
u64 mmu_enabled_at_boot __initdata;

/*
 * Standard memory resources
 */
static struct resource mem_res[] = {
	{
		.name = "Kernel code",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_SYSTEM_RAM
	},
	{
		.name = "Kernel data",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_SYSTEM_RAM
	}
};

#define kernel_code mem_res[0]
#define kernel_data mem_res[1]

/*
 * The recorded values of x0 .. x3 upon kernel entry.
 */
u64 __cacheline_aligned boot_args[4];

void __init smp_setup_processor_id(void)
{
	u64 mpidr = read_cpuid_mpidr() & MPIDR_HWID_BITMASK;
	set_cpu_logical_map(0, mpidr);

	pr_info("Booting Linux on physical CPU 0x%010lx [0x%08x]\n",
		(unsigned long)mpidr, read_cpuid_id());
}

bool arch_match_cpu_phys_id(int cpu, u64 phys_id)
{
	return phys_id == cpu_logical_map(cpu);
}

struct mpidr_hash mpidr_hash;
/**
 * smp_build_mpidr_hash - Pre-compute shifts required at each affinity
 *			  level in order to build a linear index from an
 *			  MPIDR value. Resulting algorithm is a collision
 *			  free hash carried out through shifting and ORing
 */
static void __init smp_build_mpidr_hash(void)
{
	u32 i, affinity, fs[4], bits[4], ls;
	u64 mask = 0;
	/*
	 * Pre-scan the list of MPIDRS and filter out bits that do
	 * not contribute to affinity levels, ie they never toggle.
	 */
	for_each_possible_cpu(i)
		mask |= (cpu_logical_map(i) ^ cpu_logical_map(0));
	pr_debug("mask of set bits %#llx\n", mask);
	/*
	 * Find and stash the last and first bit set at all affinity levels to
	 * check how many bits are required to represent them.
	 */
	for (i = 0; i < 4; i++) {
		affinity = MPIDR_AFFINITY_LEVEL(mask, i);
		/*
		 * Find the MSB bit and LSB bits position
		 * to determine how many bits are required
		 * to express the affinity level.
		 */
		ls = fls(affinity);
		fs[i] = affinity ? ffs(affinity) - 1 : 0;
		bits[i] = ls - fs[i];
	}
	/*
	 * An index can be created from the MPIDR_EL1 by isolating the
	 * significant bits at each affinity level and by shifting
	 * them in order to compress the 32 bits values space to a
	 * compressed set of values. This is equivalent to hashing
	 * the MPIDR_EL1 through shifting and ORing. It is a collision free
	 * hash though not minimal since some levels might contain a number
	 * of CPUs that is not an exact power of 2 and their bit
	 * representation might contain holes, eg MPIDR_EL1[7:0] = {0x2, 0x80}.
	 */
	mpidr_hash.shift_aff[0] = MPIDR_LEVEL_SHIFT(0) + fs[0];
	mpidr_hash.shift_aff[1] = MPIDR_LEVEL_SHIFT(1) + fs[1] - bits[0];
	mpidr_hash.shift_aff[2] = MPIDR_LEVEL_SHIFT(2) + fs[2] -
						(bits[1] + bits[0]);
	mpidr_hash.shift_aff[3] = MPIDR_LEVEL_SHIFT(3) +
				  fs[3] - (bits[2] + bits[1] + bits[0]);
	mpidr_hash.mask = mask;
	mpidr_hash.bits = bits[3] + bits[2] + bits[1] + bits[0];
	pr_debug("MPIDR hash: aff0[%u] aff1[%u] aff2[%u] aff3[%u] mask[%#llx] bits[%u]\n",
		mpidr_hash.shift_aff[0],
		mpidr_hash.shift_aff[1],
		mpidr_hash.shift_aff[2],
		mpidr_hash.shift_aff[3],
		mpidr_hash.mask,
		mpidr_hash.bits);
	/*
	 * 4x is an arbitrary value used to warn on a hash table much bigger
	 * than expected on most systems.
	 */
	if (mpidr_hash_size() > 4 * num_possible_cpus())
		pr_warn("Large number of MPIDR hash buckets detected\n");
}

static void __init setup_machine_fdt(phys_addr_t dt_phys)
{
	int size = 0;
	void *dt_virt = fixmap_remap_fdt(dt_phys, &size, PAGE_KERNEL);
	const char *name;

	if (dt_virt)
		memblock_reserve(dt_phys, size);

	/*
	 * dt_virt is a fixmap address, hence __pa(dt_virt) can't be used.
	 * Pass dt_phys directly.
	 */
	if (!early_init_dt_scan(dt_virt, dt_phys)) {
		pr_crit("\n"
			"Error: invalid device tree blob: PA=%pa, VA=%px, size=%d bytes\n"
			"The dtb must be 8-byte aligned and must not exceed 2 MB in size.\n"
			"\nPlease check your bootloader.\n",
			&dt_phys, dt_virt, size);

		/*
		 * Note that in this _really_ early stage we cannot even BUG()
		 * or oops, so the least terrible thing to do is cpu_relax(),
		 * or else we could end-up printing non-initialized data, etc.
		 */
		while (true)
			cpu_relax();
	}

	/* Early fixups are done, map the FDT as read-only now */
	fixmap_remap_fdt(dt_phys, &size, PAGE_KERNEL_RO);

	name = of_flat_dt_get_machine_name();
	if (!name)
		return;

	pr_info("Machine model: %s\n", name);
	dump_stack_set_arch_desc("%s (DT)", name);
}

static void __init request_standard_resources(void)
{
	struct memblock_region *region;
	struct resource *res;
	unsigned long i = 0;
	size_t res_size;

	kernel_code.start   = __pa_symbol(_text);
	kernel_code.end     = __pa_symbol(__init_begin - 1);
	kernel_data.start   = __pa_symbol(_sdata);
	kernel_data.end     = __pa_symbol(_end - 1);
	insert_resource(&iomem_resource, &kernel_code);
	insert_resource(&iomem_resource, &kernel_data);

	num_standard_resources = memblock.memory.cnt;
	res_size = num_standard_resources * sizeof(*standard_resources);
	standard_resources = memblock_alloc_or_panic(res_size, SMP_CACHE_BYTES);

	for_each_mem_region(region) {
		res = &standard_resources[i++];
		if (memblock_is_nomap(region)) {
			res->name  = "reserved";
			res->flags = IORESOURCE_MEM;
			res->start = __pfn_to_phys(memblock_region_reserved_base_pfn(region));
			res->end = __pfn_to_phys(memblock_region_reserved_end_pfn(region)) - 1;
		} else {
			res->name  = "System RAM";
			res->flags = IORESOURCE_SYSTEM_RAM | IORESOURCE_BUSY;
			res->start = __pfn_to_phys(memblock_region_memory_base_pfn(region));
			res->end = __pfn_to_phys(memblock_region_memory_end_pfn(region)) - 1;
		}

		insert_resource(&iomem_resource, res);
	}
}

static int __init reserve_memblock_reserved_regions(void)
{
	u64 i, j;

	for (i = 0; i < num_standard_resources; ++i) {
		struct resource *mem = &standard_resources[i];
		phys_addr_t r_start, r_end, mem_size = resource_size(mem);

		if (!memblock_is_region_reserved(mem->start, mem_size))
			continue;

		for_each_reserved_mem_range(j, &r_start, &r_end) {
			resource_size_t start, end;

			start = max(PFN_PHYS(PFN_DOWN(r_start)), mem->start);
			end = min(PFN_PHYS(PFN_UP(r_end)) - 1, mem->end);

			if (start > mem->end || end < mem->start)
				continue;

			reserve_region_with_split(mem, start, end, "reserved");
		}
	}

	return 0;
}
arch_initcall(reserve_memblock_reserved_regions);

u64 __cpu_logical_map[NR_CPUS] = { [0 ... NR_CPUS-1] = INVALID_HWID };

u64 cpu_logical_map(unsigned int cpu)
{
	return __cpu_logical_map[cpu];
}


#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
/* flattened device tree tags (not exported by the kernel headers) */
#define FDT_MAGIC       0xd00dfeed
#define FDT_BEGIN_NODE  1
#define FDT_END_NODE    2
#define FDT_PROP        3

/*
 * B': the DRAM size comes from the bootloader, everything else from us.
 *
 * The kernel keeps using the tree built into it -- that one carries every node
 * this port needs.  Swapping in the LK's tree loses all of them, and its dtbo
 * overlay cannot be applied to ours either: 38 of the 52 symbols that overlay
 * needs do not exist here, and fdt_overlay_apply() is atomic.
 *
 * The capacity, however, cannot be baked in -- one image has to run on the
 * 4/6/8/12 GiB units.  The only place the real size is known is the tree the
 * bootloader composed, so read /memory out of that one before switching over,
 * then trim whatever above it the embedded tree over-declared.
 */
#define EVERGO_DRAM_BASE    0x40000000ULL

/*
 * Where the bootloader says DRAM ends.
 *
 * The embedded tree has to declare *some* size, and it declares the largest
 * variant (8 GiB).  Declaring less than a unit actually has simply throws that
 * RAM away -- memblock can only learn about memory described somewhere -- while
 * declaring more only hands out addresses that do not exist.  So the tree
 * declares 8 GiB and whatever it over-declared is dropped again below, which is
 * what lets a single image serve the 4/6/8 GiB units.
 */
static u64 evergo_lk_dram_end __initdata;

/*
 * The bootloader's /reserved-memory, read back before the embedded tree
 * replaces it.
 *
 * LK carves DRAM out for the modem, SCP, SPM, RKP, MCUPM and the TEE, and puts
 * a few of those at addresses that depend on how much DRAM it trained -- the
 * RAM-dump window and the per-rank markers move between the 4/6/8 GiB units of
 * this board.  A static table in the DTS can only describe one of them, so the
 * table in mt6833-xiaomi-evergo.dts stays as the baseline and whatever LK
 * actually reserved is added on top here.  Reserving the union is safe: an
 * entry that describes DRAM this unit does not have simply marks an address
 * that memblock never knew about.
 */
#define EVERGO_LK_MAX_MBLOCK	64

struct evergo_lk_mblock {
	u64	base;
	u64	size;
};

static struct evergo_lk_mblock evergo_lk_mblocks[EVERGO_LK_MAX_MBLOCK] __initdata;
static int evergo_lk_mblocks_nr __initdata;

/*
 * Window over the bootloader's tree.
 *
 * LK does not hand over its own 170 KiB lk_main_dtb -- it applies its dtbo
 * first and the result measures 285,941 bytes (b382 measured it).  That is past
 * what early_ioremap() will map in one go, and 256 KiB -- nominally still under
 * the 260 KiB ceiling -- kills the boot outright (b383).
 *
 * In the raw LK tree and in the boot.img copy /memory sits at blob offset
 * 0x2384 (~9 KiB), but LK re-serialises the whole tree when it applies its
 * dtbo, so the layout of the 285,941-byte tree it actually hands over is not
 * the same -- a 16 KiB window found nothing (b386 "D3-nomem").
 *
 * The window is therefore 192 KiB, which b382 proved can be mapped, and the
 * walk below is hand-rolled rather than libfdt's: fdt_offset_ptr() bounds-checks
 * against fdt_totalsize() (285,941) instead of what was actually mapped, so it
 * reads past the window and takes the kernel down (b385).  The hand-written
 * walk below never leaves the window.
 */
#define EVERGO_LK_FDT_MAX   (192 * 1024)
#define EVERGO_LK_FDT_STRUCT_OFF   8    /* off_dt_struct in the FDT header */

static void __init evergo_read_lk_dram(phys_addr_t fdt_phys)
{
	u32 *p, *end;
	u32 struct_off, strings_off;
	u64 base, sz;
	void *fdt;
	int depth = 0, implausible = 0, nodes = 0;
	int in_mem = 0;
	int rm_depth = 0;	/* depth of /reserved-memory, 0 = not inside it */

	pr_info("evergo: lk dtb phys=%pa\n", &fdt_phys);

	if (!fdt_phys)
		return;

	fdt = early_memremap(fdt_phys, EVERGO_LK_FDT_MAX);
	if (!fdt) {
		return;
	}

	if (be32_to_cpup((__be32 *)fdt) != FDT_MAGIC) {
		goto out;
	}

	struct_off  = be32_to_cpup((__be32 *)((char *)fdt + 8));
	strings_off = be32_to_cpup((__be32 *)((char *)fdt + 12));

	pr_info("evergo: lk dtb struct=%#x strings=%#x totalsize=%u\n",
		struct_off, strings_off, be32_to_cpup((__be32 *)((char *)fdt + 4)));

	p = (u32 *)((char *)fdt + struct_off);
	end = (u32 *)((char *)fdt + EVERGO_LK_FDT_MAX);

	/*
	 * Walk the structure block by hand (see the note above): libfdt bounds
	 * against fdt_totalsize() rather than the mapped window and reads past it.
	 * Property names come from the strings block, which sits at ~153 KiB and
	 * therefore inside the window.
	 */
	while (p < end) {
		u32 tag = be32_to_cpup((__be32 *)p++);

		if (tag == FDT_BEGIN_NODE) {
			const char *nm = (const char *)p;
			size_t n = strlen(nm);

			depth++;
			nodes++;
			/*
			 * The root node itself has an empty name and takes depth 1, so its
			 * children -- /memory, /chosen, /reserved-memory -- are depth 2.
			 * Testing depth 1 here matched only the root and never a child,
			 * which is why 907 nodes walked and nothing was found (b388).
			 */
			in_mem = (depth == 2 &&
				  (!strcmp(nm, "memory") || !strncmp(nm, "memory@", 7)));
			if (depth == 2 && !strcmp(nm, "reserved-memory"))
				rm_depth = depth;
			p = (u32 *)(((unsigned long)p + n + 1 + 3) & ~3UL);
		} else if (tag == FDT_END_NODE) {
			if (rm_depth && depth == rm_depth)
				rm_depth = 0;
			depth--;
			in_mem = 0;
		} else if (tag == FDT_PROP) {
			u32 plen = be32_to_cpup((__be32 *)p);

			p += 2;

			/*
			 * No property names: the strings block sits at 0x3f4a4 (259 KiB),
			 * beyond the mapped window.  Identify reg by shape and content
			 * instead of position: LK prepends four properties of its own, so
			 * reg is the sixth on this board, not the second.  A 16-byte
			 * property whose base cell pair decodes to 0x40000000 is reg.
			 */
			/*
			 * Same shape-based identification as above: the strings block is
			 * out of reach, so a 16-byte property on a child of
			 * /reserved-memory can only be its reg (two address cells, two
			 * size cells).  compatible is a string, no-map is empty and
			 * phandle is one cell, so nothing else matches.
			 */
			if (rm_depth && depth == rm_depth + 1 && plen == 16) {
				base = of_read_number((const __be32 *)p, 2);
				sz = of_read_number((const __be32 *)p + 2, 2);

				if (sz && base < (1ULL << 40) && base + sz <= (1ULL << 40) &&
				    evergo_lk_mblocks_nr < EVERGO_LK_MAX_MBLOCK) {
					evergo_lk_mblocks[evergo_lk_mblocks_nr].base = base;
					evergo_lk_mblocks[evergo_lk_mblocks_nr].size = sz;
					evergo_lk_mblocks_nr++;
				}
			}

			if (in_mem && plen == 16 && !evergo_lk_dram_end) {
				base = of_read_number((const __be32 *)p, 2);
				sz = of_read_number((const __be32 *)p + 2, 2);
				if (base == EVERGO_DRAM_BASE && sz) {
					if (sz < 0x100000000ULL) {
						implausible = 1;
						break;
					}
					evergo_lk_dram_end = base + sz;
					pr_info("evergo: bootloader reports %llu MiB of DRAM\n",
						sz >> 20);
				}
			}
			p = (u32 *)(((unsigned long)p + plen + 3) & ~3UL);
		} else {
			break;
		}
	}

	if (!evergo_lk_dram_end && !implausible) {
		pr_info("evergo: no /memory in the lk dtb (%d nodes walked)\n", nodes);
	}

	pr_info("evergo: lk /reserved-memory: %d block(s) from %d nodes\n",
		evergo_lk_mblocks_nr, nodes);
	if (!rm_depth && !evergo_lk_mblocks_nr) {
		pr_warn("evergo: the lk dtb has no /reserved-memory, keeping the dts table only\n");
	}
out:
	early_memunmap(fdt, EVERGO_LK_FDT_MAX);
}

/*
 * Both trees were scanned, so memblock now spans their union -- i.e. the 8 GiB
 * the embedded tree declares.  Cut it back to what the bootloader actually
 * trained.
 */
static void __init evergo_trim_dram(void)
{
	u64 end = memblock_end_of_DRAM();

	if (!evergo_lk_dram_end || end <= evergo_lk_dram_end)
		return;

	pr_info("evergo: trimming %llu MiB the embedded tree over-declared\n",
		(end - evergo_lk_dram_end) >> 20);
	memblock_remove(evergo_lk_dram_end, end - evergo_lk_dram_end);
}

/*
 * Add whatever the bootloader reserved on top of the table compiled into the
 * embedded tree.
 *
 * This is what makes one image fit every DRAM variant of this board: the DTS
 * carries the layout measured on one unit, and anything this unit's LK placed
 * somewhere else -- the RAM-dump window, the per-rank markers, the modem
 * mblocks -- is picked up here at run time.  Reserving a range twice is
 * harmless; reserving one that describes absent DRAM just marks an address
 * memblock never had.
 *
 * no-map matters: these ranges belong to firmware that keeps running, so they
 * must not be mapped into the linear map either.  memblock_mark_nomap() requires
 * the range to be reserved first, hence the order.
 */
static void __init evergo_apply_lk_reserved(void)
{
	phys_addr_t kstart = __pa_symbol(_text);
	phys_addr_t kend = __pa_symbol(_end);
	u64 dram_end = evergo_lk_dram_end ? evergo_lk_dram_end
					  : EVERGO_DRAM_BASE + (8ULL << 30);
	int i, added = 0;

	for (i = 0; i < evergo_lk_mblocks_nr; i++) {
		u64 base = evergo_lk_mblocks[i].base;
		u64 size = evergo_lk_mblocks[i].size;
		u64 end = base + size;

		/*
		 * Filter hard before touching memblock.  reg is identified by shape
		 * alone (the strings block is out of reach), so a mis-parse is
		 * possible, and handing memblock -- worse, memblock_mark_nomap() -- a
		 * range that covers the running kernel or a large part of DRAM kills
		 * the boot inside paging_init() with nothing printed at all (b398).
		 */
		if (base < EVERGO_DRAM_BASE || !size || size > (1ULL << 29) ||
		    end < base || end > dram_end) {
			continue;
		}

		/* the kernel is running from here -- never reserve or unmap it */
		if (base < kend && end > kstart) {
			continue;
		}

		/*
		 * The DTS table already reserved and nomap'd its ranges; adding them
		 * again buys nothing, and memblock_is_reserved() on the base address is
		 * enough to tell.
		 */
		if (memblock_is_reserved(base)) {
			continue;
		}

		/*
		 * Remove rather than reserve+nomap.
		 *
		 * memblock_mark_nomap() only suppresses the linear mapping; it does not
		 * keep the range out of the allocator.
		 * early_pgtable_alloc() allocates with
		 * memblock_phys_alloc_range(..., MEMBLOCK_ALLOC_NOLEAKTRACE), which
		 * filters reserved ranges but not NOMAP ones, so a page-table page could
		 * be handed out of a nomap range -- and then the kernel faults the
		 * moment it touches it through __va(), with the linear mapping
		 * deliberately absent.  That is what killed paging_init() (b400:
		 * the last mark was EVM:SA2-paging and the new 128 MiB range at
		 * 0x49100000 was in the pool).
		 *
		 * Taking the range out of memblock.memory does both jobs at once: it
		 * can no longer be allocated and it is no longer mapped.
		 */
		if (memblock_remove(base, size)) {
			continue;
		}
		added++;
	}

	pr_info("evergo: kept %d of %d bootloader reservations\n",
		added, evergo_lk_mblocks_nr);
}
#endif

void __init __no_sanitize_address setup_arch(char **cmdline_p)
{
	setup_initial_init_mm(_text, _etext, _edata, _end);

	*cmdline_p = boot_command_line;

	kaslr_init();

	early_fixmap_init();
	early_ioremap_init();

#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
	/* read DRAM size / reserved ranges out of the LK tree before we swap it */
	evergo_read_lk_dram(__fdt_pointer);

	/*
	 * Bring-up aid: the LK of the evergo composes the device tree itself
	 * (its display/lcm code needs the vendor panel data), so the one it
	 * hands over is not the one we want. Use the DT embedded through
	 * CONFIG_EXTRA_FIRMWARE instead; it is found without any allocation,
	 * which is what firmware_request_builtin() is meant for.
	 */
	{
		struct firmware fw;

		if (firmware_request_builtin(&fw, "mt6833-xiaomi-evergo.dtb")) {
			__fdt_pointer = __pa(fw.data);
			pr_info("evergo: using embedded device tree (%zu bytes)\n",
				fw.size);
		} else {
			pr_warn("evergo: embedded device tree missing, keeping the bootloader one\n");
		}
	}
#endif

	setup_machine_fdt(__fdt_pointer);
#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
	evergo_trim_dram();
#endif

	/*
	 * Initialise the static keys early as they may be enabled by the
	 * cpufeature code and early parameters.
	 */
	jump_label_init();
	parse_early_param();

	dynamic_scs_init();

	/*
	 * The primary CPU enters the kernel with all DAIF exceptions masked.
	 *
	 * We must unmask Debug and SError before preemption or scheduling is
	 * possible to ensure that these are consistently unmasked across
	 * threads, and we want to unmask SError as soon as possible after
	 * initializing earlycon so that we can report any SErrors immediately.
	 *
	 * IRQ and FIQ will be unmasked after the root irqchip has been
	 * detected and initialized.
	 */
	local_daif_restore(DAIF_PROCCTX_NOIRQ);

	/*
	 * TTBR0 is only used for the identity mapping at this stage. Make it
	 * point to zero page to avoid speculatively fetching new entries.
	 */
	cpu_uninstall_idmap();

	xen_early_init();
	efi_init();

	if (!efi_enabled(EFI_BOOT)) {
		if ((u64)_text % MIN_KIMG_ALIGN)
			pr_warn(FW_BUG "Kernel image misaligned at boot, please fix your bootloader!");
		WARN_TAINT(mmu_enabled_at_boot, TAINT_FIRMWARE_WORKAROUND,
			   FW_BUG "Booted with MMU enabled!");
	}

	arm64_memblock_init();

	/*
	 * Only now, after arm64_memblock_init() has folded the embedded tree's
	 * reserved-memory into memblock.  Doing this earlier is not equivalent:
	 * the extra memblock_reserve() calls push memblock.reserved into a new
	 * array, the allocation for that array splits memblock.memory, and the
	 * DTS nodes that follow can then no longer be marked no-map.
	 */
#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
	evergo_apply_lk_reserved();
#endif

	paging_init();

	acpi_table_upgrade();

	/* Parse the ACPI tables for possible boot-time configuration */
	acpi_boot_table_init();

	if (acpi_disabled)
		unflatten_device_tree();

	bootmem_init();

	kasan_init();

	request_standard_resources();

	if (acpi_disabled)
		psci_dt_init();
	else
		psci_acpi_init();

	arm64_rsi_init();

	init_bootcpu_ops();
	smp_init_cpus();
	smp_build_mpidr_hash();

#ifdef CONFIG_ARM64_SW_TTBR0_PAN
	/*
	 * Make sure init_thread_info.ttbr0 always generates translation
	 * faults in case uaccess_enable() is inadvertently called by the init
	 * thread.
	 */
	init_task.thread_info.ttbr0 = phys_to_ttbr(__pa_symbol(reserved_pg_dir));
#endif

	if (boot_args[1] || boot_args[2] || boot_args[3]) {
		pr_err("WARNING: x1-x3 nonzero in violation of boot protocol:\n"
			"\tx1: %016llx\n\tx2: %016llx\n\tx3: %016llx\n"
			"This indicates a broken bootloader or old kernel\n",
			boot_args[1], boot_args[2], boot_args[3]);
	}
}

static inline bool cpu_can_disable(unsigned int cpu)
{
#ifdef CONFIG_HOTPLUG_CPU
	const struct cpu_operations *ops = get_cpu_ops(cpu);

	if (ops && ops->cpu_can_disable)
		return ops->cpu_can_disable(cpu);
#endif
	return false;
}

bool arch_cpu_is_hotpluggable(int num)
{
	return cpu_can_disable(num);
}

static void dump_kernel_offset(void)
{
	const unsigned long offset = kaslr_offset();

	if (IS_ENABLED(CONFIG_RANDOMIZE_BASE) && offset > 0) {
		pr_emerg("Kernel Offset: 0x%lx from 0x%lx\n",
			 offset, KIMAGE_VADDR);
		pr_emerg("PHYS_OFFSET: 0x%llx\n", PHYS_OFFSET);
	} else {
		pr_emerg("Kernel Offset: disabled\n");
	}
}

static int arm64_panic_block_dump(struct notifier_block *self,
				  unsigned long v, void *p)
{
	dump_kernel_offset();
	dump_cpu_features();
	dump_mem_limit();
	return 0;
}

static struct notifier_block arm64_panic_block = {
	.notifier_call = arm64_panic_block_dump
};

static int __init register_arm64_panic_block(void)
{
	atomic_notifier_chain_register(&panic_notifier_list,
				       &arm64_panic_block);
	return 0;
}
device_initcall(register_arm64_panic_block);

static int __init check_mmu_enabled_at_boot(void)
{
	if (!efi_enabled(EFI_BOOT) && mmu_enabled_at_boot)
		panic("Non-EFI boot detected with MMU and caches enabled");
	return 0;
}
device_initcall_sync(check_mmu_enabled_at_boot);
