	.arch armv8-a
	.file	"sunxi_awlink_asm.c"

	.text
	.align	2
	.type	pmd_to_page, %function
pmd_to_page:
	hint	25 // paciasp
	sub	sp, sp, #48	//,,
	str	x0, [sp, 8]	// pmd, pmd
	mov	x0, -4096	// tmp98,
	str	x0, [sp, 24]	// tmp98, mask
	ldr	x1, [sp, 8]	// pmd.0_1, pmd
	ldr	x0, [sp, 24]	// tmp99, mask
	and	x1, x1, x0	// _2, pmd.0_1, tmp99
	mov	x0, 549755813888	// tmp100,
	add	x0, x1, x0	// _3, _2, tmp100
	lsr	x0, x0, 12	// tmp101, _3,
	str	x0, [sp, 32]	// tmp101, __idx
	ldr	x0, [sp, 32]	// tmp102, __idx
	lsl	x1, x0, 6	// _4, tmp102,
	mov	x0, -8589934592	// tmp104,
	add	x0, x1, x0	// tmp103, _4, tmp104
	str	x0, [sp, 40]	// tmp103, __addr
	ldr	x0, [sp, 40]	// _9, __addr
	add	sp, sp, 48	//,,
	hint	29 // autiasp
	ret	
	.size	pmd_to_page, .-pmd_to_page
	.align	2
	.type	log_write_mmio, %function
log_write_mmio:
	hint	25 // paciasp
	sub	sp, sp, #32	//,,
	str	x0, [sp, 24]	// val, val
	strb	w1, [sp, 23]	// width, width
	str	x2, [sp, 8]	// addr, addr
	str	x3, [sp]	// caller_addr, caller_addr
	nop	
	add	sp, sp, 32	//,,
	hint	29 // autiasp
	ret	
	.size	log_write_mmio, .-log_write_mmio
	.align	2
	.type	log_post_write_mmio, %function
log_post_write_mmio:
	hint	25 // paciasp
	sub	sp, sp, #32	//,,
	str	x0, [sp, 24]	// val, val
	strb	w1, [sp, 23]	// width, width
	str	x2, [sp, 8]	// addr, addr
	str	x3, [sp]	// caller_addr, caller_addr
	nop	
	add	sp, sp, 32	//,,
	hint	29 // autiasp
	ret	
	.size	log_post_write_mmio, .-log_post_write_mmio
	.align	2
	.type	log_read_mmio, %function
log_read_mmio:
	hint	25 // paciasp
	sub	sp, sp, #32	//,,
	strb	w0, [sp, 31]	// width, width
	str	x1, [sp, 16]	// addr, addr
	str	x2, [sp, 8]	// caller_addr, caller_addr
	nop	
	add	sp, sp, 32	//,,
	hint	29 // autiasp
	ret	
	.size	log_read_mmio, .-log_read_mmio
	.align	2
	.type	log_post_read_mmio, %function
log_post_read_mmio:
	hint	25 // paciasp
	sub	sp, sp, #32	//,,
	str	x0, [sp, 24]	// val, val
	strb	w1, [sp, 23]	// width, width
	str	x2, [sp, 8]	// addr, addr
	str	x3, [sp]	// caller_addr, caller_addr
	nop	
	add	sp, sp, 32	//,,
	hint	29 // autiasp
	ret	
	.size	log_post_read_mmio, .-log_post_read_mmio
	.align	2
	.type	readl, %function
readl:
	hint	25 // paciasp
	stp	x29, x30, [sp, -64]!	//,,,
	mov	x29, sp	//,
	str	x0, [sp, 24]	// addr, addr
.L8:
	adrp	x0, .L8	// tmp99,
	add	x0, x0, :lo12:.L8	// _3, tmp99,
	mov	x2, x0	//, _3
	ldr	x1, [sp, 24]	//, addr
	mov	w0, 32	//,
	bl	log_read_mmio		//
	ldr	x0, [sp, 24]	// tmp100, addr
	str	x0, [sp, 56]	// tmp100, addr
	ldr	x0, [sp, 56]	// tmp102, addr
#APP
	.if 1 == 1
661:
	ldr w0, [x0]	// val, tmp102
662:
.pushsection .altinstructions,"a"
 .word 661b - .
 .word 663f - .
 .hword 61
 .byte 662b-661b
 .byte 664f-663f
.popsection
.subsection 1
663:
	ldar w0, [x0]	// val, tmp102
664:
	.org	. - (664b-663b) + (662b-661b)
	.org	. - (662b-661b) + (664b-663b)
	.previous
.endif

// 0 "" 2
#NO_APP
	str	w0, [sp, 44]	// val, val
	ldr	w0, [sp, 44]	// D.69895, val
	str	w0, [sp, 40]	// D.69895, val
#APP
	dmb oshld
// 0 "" 2
#NO_APP
	ldr	w0, [sp, 40]	// _1, val
#APP
	eor	x0, x0, x0	// tmp, _1
cbnz	x0, .	// tmp
// 0 "" 2
#NO_APP
	str	x0, [sp, 48]	// tmp, tmp
	ldr	w4, [sp, 40]	// _2, val
.L10:
	adrp	x0, .L10	// tmp104,
	add	x0, x0, :lo12:.L10	// _11, tmp104,
	mov	x3, x0	//, _11
	ldr	x2, [sp, 24]	//, addr
	mov	w1, 32	//,
	mov	x0, x4	//, _2
	bl	log_post_read_mmio		//
	ldr	w0, [sp, 40]	// _13, val
	ldp	x29, x30, [sp], 64	//,,,
	hint	29 // autiasp
	ret	
	.size	readl, .-readl
	.align	2
	.type	writel, %function
writel:
	hint	25 // paciasp
	stp	x29, x30, [sp, -48]!	//,,,
	mov	x29, sp	//,
	str	w0, [sp, 28]	// value, value
	str	x1, [sp, 16]	// addr, addr
	ldr	w4, [sp, 28]	// _1, value
.L13:
	adrp	x0, .L13	// tmp96,
	add	x0, x0, :lo12:.L13	// _4, tmp96,
	mov	x3, x0	//, _4
	ldr	x2, [sp, 16]	//, addr
	mov	w1, 32	//,
	mov	x0, x4	//, _1
	bl	log_write_mmio		//
#APP
	dmb oshst
// 0 "" 2
#NO_APP
	ldr	w0, [sp, 28]	// tmp97, value
	str	w0, [sp, 36]	// tmp97, val
	ldr	x0, [sp, 16]	// tmp98, addr
	str	x0, [sp, 40]	// tmp98, addr
	ldr	w0, [sp, 36]	// tmp99, val
	ldr	x1, [sp, 40]	// tmp100, addr
#APP
	str w0, [x1]	// tmp99, tmp100
// 0 "" 2
#NO_APP
	nop	
	ldr	w4, [sp, 28]	// _2, value
.L14:
	adrp	x0, .L14	// tmp101,
	add	x0, x0, :lo12:.L14	// _9, tmp101,
	mov	x3, x0	//, _9
	ldr	x2, [sp, 16]	//, addr
	mov	w1, 32	//,
	mov	x0, x4	//, _2
	bl	log_post_write_mmio		//
	nop	
	ldp	x29, x30, [sp], 48	//,,,
	hint	29 // autiasp
	ret	
	.size	writel, .-writel
	.section	.rodata
	.align	3
.LC0:
	.string	"unknown"
	.align	3
.LC1:
	.string	"firmware"
	.align	3
.LC2:
	.string	"kernel-module"
	.align	3
.LC3:
	.string	"kexec-image"
	.align	3
.LC4:
	.string	"kexec-initramfs"
	.align	3
.LC5:
	.string	"security-policy"
	.align	3
.LC6:
	.string	"x509-certificate"
	.align	3
.LC7:
	.string	""
	.align	3
	.type	kernel_read_file_str, %object
	.size	kernel_read_file_str, 64
kernel_read_file_str:
	.xword	.LC0
	.xword	.LC1
	.xword	.LC2
	.xword	.LC3
	.xword	.LC4
	.xword	.LC5
	.xword	.LC6
	.xword	.LC7
	.align	3
	.type	kernel_load_data_str, %object
	.size	kernel_load_data_str, 64
kernel_load_data_str:
	.xword	.LC0
	.xword	.LC1
	.xword	.LC2
	.xword	.LC3
	.xword	.LC4
	.xword	.LC5
	.xword	.LC6
	.xword	.LC7
	.align	1
	.type	eth_reserved_addr_base, %object
	.size	eth_reserved_addr_base, 6
eth_reserved_addr_base:
	.string	"\001\200\302"
	.string	""
	.string	""
	.text
	.align	2
	.type	junk_fun1, %function
junk_fun1:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 15	// tmp100,
	str	w0, [sp, 8]	// tmp100, mod_reg_val
	mov	w0, 10	// tmp101,
	str	w0, [sp, 12]	// tmp101, mod_reg_addr
	ldr	w1, [sp, 8]	// mod_reg_val.1_1, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.2_2, mod_reg_addr
	cmp	w1, w0	// mod_reg_val.1_1, mod_reg_addr.2_2
	bge	.L16		//,
	ldr	w0, [sp, 12]	// _6, mod_reg_addr
	b	.L17		//
.L16:
	ldr	w0, [sp, 12]	// mod_reg_addr.3_3, mod_reg_addr
	cmp	w0, 16	// mod_reg_addr.3_3,
	bne	.L18		//,
	ldr	w0, [sp, 8]	// _6, mod_reg_val
	b	.L17		//
.L18:
	ldr	w1, [sp, 12]	// mod_reg_addr.4_11, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.5_4, mod_reg_val
	eor	w0, w1, w0	// _5, mod_reg_addr.4_11, mod_reg_val.5_4
	str	w0, [sp, 8]	// _5, mod_reg_val
	ldr	w0, [sp, 8]	// _6, mod_reg_val
.L17:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun1, .-junk_fun1
	.align	2
	.type	junk_fun2, %function
junk_fun2:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 2	// tmp104,
	str	w0, [sp, 8]	// tmp104, mod_reg_val
	mov	w0, 2	// tmp105,
	str	w0, [sp, 12]	// tmp105, mod_reg_addr
	ldr	w0, [sp, 12]	// mod_reg_addr.6_1, mod_reg_addr
	and	w0, w0, 1	// _2, mod_reg_addr.6_1,
	cmp	w0, 768	// _2,
	beq	.L20		//,
	ldr	w1, [sp, 8]	// mod_reg_val.7_15, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.8_3, mod_reg_addr
	add	w0, w1, w0	// _4, mod_reg_val.7_15, mod_reg_addr.8_3
	str	w0, [sp, 12]	// _4, mod_reg_addr
.L20:
	ldr	w1, [sp, 12]	// mod_reg_addr.9_5, mod_reg_addr
	mov	w0, 819	// tmp106,
	mul	w0, w1, w0	// _6, mod_reg_addr.9_5, tmp106
	str	w0, [sp, 8]	// _6, mod_reg_val
	ldr	w0, [sp, 8]	// mod_reg_val.10_7, mod_reg_val
	and	w0, w0, 2	// _8, mod_reg_val.10_7,
	str	w0, [sp, 12]	// _8, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.11_9, mod_reg_val
	cmp	w0, 819	// mod_reg_val.11_9,
	bne	.L21		//,
	ldr	w0, [sp, 12]	// _10, mod_reg_addr
	b	.L22		//
.L21:
	ldr	w0, [sp, 8]	// _10, mod_reg_val
.L22:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun2, .-junk_fun2
	.align	2
	.type	junk_fun3, %function
junk_fun3:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 11	// tmp99,
	str	w0, [sp, 8]	// tmp99, mod_reg_val
	mov	w0, 5	// tmp100,
	str	w0, [sp, 12]	// tmp100, mod_reg_addr
	ldr	w0, [sp, 12]	// mod_reg_addr.12_1, mod_reg_addr
	and	w0, w0, 1	// _2, mod_reg_addr.12_1,
	cmp	w0, 768	// _2,
	beq	.L24		//,
	ldr	w1, [sp, 8]	// mod_reg_val.13_9, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.14_3, mod_reg_addr
	add	w0, w1, w0	// _4, mod_reg_val.13_9, mod_reg_addr.14_3
	str	w0, [sp, 12]	// _4, mod_reg_addr
.L24:
	ldr	w0, [sp, 8]	// _11, mod_reg_val
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun3, .-junk_fun3
	.align	2
	.type	junk_fun4, %function
junk_fun4:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 10	// tmp99,
	str	w0, [sp, 8]	// tmp99, mod_reg_val
	mov	w0, 7	// tmp100,
	str	w0, [sp, 12]	// tmp100, mod_reg_addr
	ldr	w1, [sp, 12]	// mod_reg_addr.15_1, mod_reg_addr
	mov	w0, 819	// tmp101,
	mul	w0, w1, w0	// _2, mod_reg_addr.15_1, tmp101
	str	w0, [sp, 8]	// _2, mod_reg_val
	ldr	w1, [sp, 12]	// mod_reg_addr.16_9, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.17_3, mod_reg_val
	add	w0, w1, w0	// _4, mod_reg_addr.16_9, mod_reg_val.17_3
	str	w0, [sp, 8]	// _4, mod_reg_val
	ldr	w0, [sp, 8]	// _11, mod_reg_val
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun4, .-junk_fun4
	.align	2
	.type	junk_fun5, %function
junk_fun5:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 1	// tmp101,
	str	w0, [sp, 8]	// tmp101, mod_reg_val
	mov	w0, 1	// tmp102,
	str	w0, [sp, 12]	// tmp102, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.18_1, mod_reg_val
	cmp	w0, 7	// mod_reg_val.18_1,
	bne	.L29		//,
	ldr	w0, [sp, 12]	// _6, mod_reg_addr
	b	.L30		//
.L29:
	ldr	w1, [sp, 8]	// mod_reg_val.19_11, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.20_2, mod_reg_addr
	eor	w0, w1, w0	// _3, mod_reg_val.19_11, mod_reg_addr.20_2
	str	w0, [sp, 12]	// _3, mod_reg_addr
	ldr	w1, [sp, 12]	// mod_reg_addr.21_13, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.22_4, mod_reg_val
	add	w0, w1, w0	// _5, mod_reg_addr.21_13, mod_reg_val.22_4
	str	w0, [sp, 8]	// _5, mod_reg_val
	ldr	w0, [sp, 8]	// _6, mod_reg_val
.L30:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun5, .-junk_fun5
	.align	2
	.type	junk_fun6, %function
junk_fun6:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 15	// tmp111,
	str	w0, [sp, 8]	// tmp111, mod_reg_val
	mov	w0, 1	// tmp112,
	str	w0, [sp, 12]	// tmp112, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.23_1, mod_reg_val
	cmp	w0, 16	// mod_reg_val.23_1,
	bne	.L32		//,
	ldr	w0, [sp, 12]	// _17, mod_reg_addr
	b	.L33		//
.L32:
	ldr	w0, [sp, 8]	// mod_reg_val.24_2, mod_reg_val
	add	w1, w0, 1	// _4, mod_reg_val.24_2,
	str	w1, [sp, 8]	// _4, mod_reg_val
	and	w0, w0, 8	// _5, mod_reg_val.24_2,
	cmp	w0, 0	// _5,
	bne	.L34		//,
	ldr	w1, [sp, 8]	// mod_reg_val.26_25, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.27_6, mod_reg_addr
	mul	w0, w1, w0	// _7, mod_reg_val.26_25, mod_reg_addr.27_6
	str	w0, [sp, 12]	// _7, mod_reg_addr
.L34:
	ldr	w1, [sp, 12]	// mod_reg_addr.28_27, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.29_8, mod_reg_val
	add	w0, w1, w0	// _9, mod_reg_addr.28_27, mod_reg_val.29_8
	str	w0, [sp, 8]	// _9, mod_reg_val
	ldr	w0, [sp, 8]	// mod_reg_val.30_10, mod_reg_val
	add	w1, w0, 1	// _12, mod_reg_val.30_10,
	str	w1, [sp, 8]	// _12, mod_reg_val
	and	w0, w0, 8	// _13, mod_reg_val.30_10,
	cmp	w0, 0	// _13,
	bne	.L35		//,
	ldr	w1, [sp, 8]	// mod_reg_val.32_30, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.33_14, mod_reg_addr
	mul	w0, w1, w0	// _15, mod_reg_val.32_30, mod_reg_addr.33_14
	str	w0, [sp, 12]	// _15, mod_reg_addr
.L35:
	ldr	w0, [sp, 8]	// mod_reg_val.34_16, mod_reg_val
	cmp	w0, 7	// mod_reg_val.34_16,
	bne	.L36		//,
	ldr	w0, [sp, 12]	// _17, mod_reg_addr
	b	.L33		//
.L36:
	ldr	w0, [sp, 8]	// _17, mod_reg_val
.L33:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun6, .-junk_fun6
	.align	2
	.type	junk_fun7, %function
junk_fun7:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 11	// tmp101,
	str	w0, [sp, 8]	// tmp101, mod_reg_val
	mov	w0, 13	// tmp102,
	str	w0, [sp, 12]	// tmp102, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.35_1, mod_reg_val
	add	w1, w0, 1	// _3, mod_reg_val.35_1,
	str	w1, [sp, 8]	// _3, mod_reg_val
	and	w0, w0, 8	// _4, mod_reg_val.35_1,
	cmp	w0, 0	// _4,
	bne	.L38		//,
	ldr	w1, [sp, 8]	// mod_reg_val.37_14, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.38_5, mod_reg_addr
	mul	w0, w1, w0	// _6, mod_reg_val.37_14, mod_reg_addr.38_5
	str	w0, [sp, 12]	// _6, mod_reg_addr
.L38:
	ldr	w0, [sp, 8]	// mod_reg_val.39_7, mod_reg_val
	cmp	w0, 7	// mod_reg_val.39_7,
	bne	.L39		//,
	ldr	w0, [sp, 12]	// _8, mod_reg_addr
	b	.L40		//
.L39:
	ldr	w0, [sp, 8]	// _8, mod_reg_val
.L40:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun7, .-junk_fun7
	.align	2
	.type	junk_fun8, %function
junk_fun8:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 2	// tmp100,
	str	w0, [sp, 8]	// tmp100, mod_reg_val
	mov	w0, 2	// tmp101,
	str	w0, [sp, 12]	// tmp101, mod_reg_addr
	ldr	w1, [sp, 8]	// mod_reg_val.40_1, mod_reg_val
	mov	w0, 243	// tmp102,
	and	w0, w1, w0	// _2, mod_reg_val.40_1, tmp102
	str	w0, [sp, 12]	// _2, mod_reg_addr
	ldr	w1, [sp, 8]	// mod_reg_val.41_11, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.42_3, mod_reg_addr
	eor	w0, w1, w0	// _4, mod_reg_val.41_11, mod_reg_addr.42_3
	str	w0, [sp, 12]	// _4, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.43_5, mod_reg_val
	cmp	w0, 16	// mod_reg_val.43_5,
	bne	.L42		//,
	ldr	w0, [sp, 12]	// _6, mod_reg_addr
	b	.L43		//
.L42:
	ldr	w0, [sp, 8]	// _6, mod_reg_val
.L43:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun8, .-junk_fun8
	.align	2
	.type	junk_fun9, %function
junk_fun9:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 15	// tmp108,
	str	w0, [sp, 8]	// tmp108, mod_reg_val
	mov	w0, 12	// tmp109,
	str	w0, [sp, 12]	// tmp109, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.44_1, mod_reg_val
	add	w0, w0, 1	// _2, mod_reg_val.44_1,
	str	w0, [sp, 8]	// _2, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.45_3, mod_reg_addr
	add	w0, w0, 51	// _4, mod_reg_addr.45_3,
	str	w0, [sp, 12]	// _4, mod_reg_addr
	ldr	w0, [sp, 12]	// mod_reg_addr.46_5, mod_reg_addr
	cmp	w0, 7	// mod_reg_addr.46_5,
	bne	.L45		//,
	ldr	w0, [sp, 12]	// _15, mod_reg_addr
	b	.L46		//
.L45:
	ldr	w0, [sp, 8]	// mod_reg_val.47_6, mod_reg_val
	add	w1, w0, 1	// _8, mod_reg_val.47_6,
	str	w1, [sp, 8]	// _8, mod_reg_val
	and	w0, w0, 8	// _9, mod_reg_val.47_6,
	cmp	w0, 0	// _9,
	bne	.L47		//,
	ldr	w1, [sp, 8]	// mod_reg_val.49_24, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.50_10, mod_reg_addr
	mul	w0, w1, w0	// _11, mod_reg_val.49_24, mod_reg_addr.50_10
	str	w0, [sp, 12]	// _11, mod_reg_addr
.L47:
	ldr	w0, [sp, 12]	// mod_reg_addr.51_12, mod_reg_addr
	cmp	w0, 16	// mod_reg_addr.51_12,
	bne	.L48		//,
	ldr	w0, [sp, 8]	// _15, mod_reg_val
	b	.L46		//
.L48:
	ldr	w1, [sp, 8]	// mod_reg_val.52_13, mod_reg_val
	mov	w0, 243	// tmp110,
	and	w0, w1, w0	// _14, mod_reg_val.52_13, tmp110
	str	w0, [sp, 12]	// _14, mod_reg_addr
	ldr	w0, [sp, 8]	// _15, mod_reg_val
.L46:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun9, .-junk_fun9
	.align	2
	.type	junk_fun10, %function
junk_fun10:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 9	// tmp101,
	str	w0, [sp, 8]	// tmp101, mod_reg_val
	mov	w0, 11	// tmp102,
	str	w0, [sp, 12]	// tmp102, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.53_1, mod_reg_val
	add	w0, w0, 1	// _2, mod_reg_val.53_1,
	str	w0, [sp, 8]	// _2, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.54_3, mod_reg_addr
	add	w0, w0, 51	// _4, mod_reg_addr.54_3,
	str	w0, [sp, 12]	// _4, mod_reg_addr
	ldr	w0, [sp, 12]	// mod_reg_addr.55_5, mod_reg_addr
	cmp	w0, 7	// mod_reg_addr.55_5,
	bne	.L50		//,
	ldr	w0, [sp, 12]	// _8, mod_reg_addr
	b	.L51		//
.L50:
	ldr	w1, [sp, 8]	// mod_reg_val.56_6, mod_reg_val
	mov	w0, 243	// tmp103,
	and	w0, w1, w0	// _7, mod_reg_val.56_6, tmp103
	str	w0, [sp, 12]	// _7, mod_reg_addr
	ldr	w0, [sp, 8]	// _8, mod_reg_val
.L51:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun10, .-junk_fun10
	.align	2
	.type	junk_fun11, %function
junk_fun11:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 4	// tmp95,
	str	w0, [sp, 8]	// tmp95, mod_reg_val
	mov	w0, 11	// tmp96,
	str	w0, [sp, 12]	// tmp96, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.57_1, mod_reg_val
	cmp	w0, 16	// mod_reg_val.57_1,
	bne	.L53		//,
	ldr	w0, [sp, 12]	// _2, mod_reg_addr
	b	.L54		//
.L53:
	ldr	w0, [sp, 8]	// _2, mod_reg_val
.L54:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun11, .-junk_fun11
	.align	2
	.type	junk_fun12, %function
junk_fun12:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 6	// tmp103,
	str	w0, [sp, 8]	// tmp103, mod_reg_val
	mov	w0, 11	// tmp104,
	str	w0, [sp, 12]	// tmp104, mod_reg_addr
	ldr	w1, [sp, 12]	// mod_reg_addr.58_12, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.59_1, mod_reg_val
	eor	w0, w1, w0	// _2, mod_reg_addr.58_12, mod_reg_val.59_1
	str	w0, [sp, 8]	// _2, mod_reg_val
	ldr	w1, [sp, 12]	// mod_reg_addr.60_3, mod_reg_addr
	mov	w0, 819	// tmp105,
	mul	w0, w1, w0	// _4, mod_reg_addr.60_3, tmp105
	str	w0, [sp, 8]	// _4, mod_reg_val
	ldr	w1, [sp, 12]	// mod_reg_addr.61_15, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.62_5, mod_reg_val
	eor	w0, w1, w0	// _6, mod_reg_addr.61_15, mod_reg_val.62_5
	str	w0, [sp, 8]	// _6, mod_reg_val
	ldr	w0, [sp, 8]	// mod_reg_val.63_7, mod_reg_val
	cmp	w0, 7	// mod_reg_val.63_7,
	bne	.L56		//,
	ldr	w0, [sp, 12]	// _8, mod_reg_addr
	b	.L57		//
.L56:
	ldr	w0, [sp, 8]	// _8, mod_reg_val
.L57:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun12, .-junk_fun12
	.align	2
	.type	junk_fun13, %function
junk_fun13:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 8	// tmp111,
	str	w0, [sp, 8]	// tmp111, mod_reg_val
	mov	w0, 15	// tmp112,
	str	w0, [sp, 12]	// tmp112, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.64_1, mod_reg_val
	cmp	w0, 819	// mod_reg_val.64_1,
	bne	.L59		//,
	ldr	w0, [sp, 12]	// _16, mod_reg_addr
	b	.L60		//
.L59:
	ldr	w1, [sp, 8]	// mod_reg_val.65_2, mod_reg_val
	mov	w0, 243	// tmp113,
	and	w0, w1, w0	// _3, mod_reg_val.65_2, tmp113
	str	w0, [sp, 12]	// _3, mod_reg_addr
	ldr	w1, [sp, 12]	// mod_reg_addr.66_24, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.67_4, mod_reg_val
	add	w0, w1, w0	// _5, mod_reg_addr.66_24, mod_reg_val.67_4
	str	w0, [sp, 8]	// _5, mod_reg_val
	ldr	w0, [sp, 8]	// mod_reg_val.68_6, mod_reg_val
	add	w1, w0, 1	// _8, mod_reg_val.68_6,
	str	w1, [sp, 8]	// _8, mod_reg_val
	and	w0, w0, 8	// _9, mod_reg_val.68_6,
	cmp	w0, 0	// _9,
	bne	.L61		//,
	ldr	w1, [sp, 8]	// mod_reg_val.70_27, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.71_10, mod_reg_addr
	mul	w0, w1, w0	// _11, mod_reg_val.70_27, mod_reg_addr.71_10
	str	w0, [sp, 12]	// _11, mod_reg_addr
.L61:
	ldr	w0, [sp, 12]	// mod_reg_addr.72_12, mod_reg_addr
	and	w0, w0, 1	// _13, mod_reg_addr.72_12,
	cmp	w0, 768	// _13,
	beq	.L62		//,
	ldr	w1, [sp, 8]	// mod_reg_val.73_29, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.74_14, mod_reg_addr
	add	w0, w1, w0	// _15, mod_reg_val.73_29, mod_reg_addr.74_14
	str	w0, [sp, 12]	// _15, mod_reg_addr
.L62:
	ldr	w0, [sp, 8]	// _16, mod_reg_val
.L60:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun13, .-junk_fun13
	.align	2
	.type	junk_fun14, %function
junk_fun14:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 4	// tmp107,
	str	w0, [sp, 8]	// tmp107, mod_reg_val
	mov	w0, 7	// tmp108,
	str	w0, [sp, 12]	// tmp108, mod_reg_addr
	ldr	w1, [sp, 8]	// mod_reg_val.75_1, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.76_2, mod_reg_addr
	cmp	w1, w0	// mod_reg_val.75_1, mod_reg_addr.76_2
	bge	.L64		//,
	ldr	w1, [sp, 8]	// mod_reg_val.77_3, mod_reg_val
	mov	w0, w1	// tmp109, mod_reg_val.77_3
	lsl	w0, w0, 1	// tmp110, tmp109,
	add	w1, w0, w1	// _4, tmp109, mod_reg_val.77_3
	ldr	w0, [sp, 12]	// mod_reg_addr.78_5, mod_reg_addr
	add	w0, w1, w0	// _6, _4, mod_reg_addr.78_5
	str	w0, [sp, 12]	// _6, mod_reg_addr
.L64:
	ldr	w0, [sp, 12]	// mod_reg_addr.79_7, mod_reg_addr
	and	w0, w0, 1	// _8, mod_reg_addr.79_7,
	cmp	w0, 768	// _8,
	beq	.L65		//,
	ldr	w1, [sp, 8]	// mod_reg_val.80_20, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.81_9, mod_reg_addr
	add	w0, w1, w0	// _10, mod_reg_val.80_20, mod_reg_addr.81_9
	str	w0, [sp, 12]	// _10, mod_reg_addr
.L65:
	ldr	w1, [sp, 8]	// mod_reg_val.82_11, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.83_12, mod_reg_addr
	cmp	w1, w0	// mod_reg_val.82_11, mod_reg_addr.83_12
	bge	.L66		//,
	ldr	w0, [sp, 12]	// _13, mod_reg_addr
	b	.L67		//
.L66:
	ldr	w0, [sp, 8]	// _13, mod_reg_val
.L67:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun14, .-junk_fun14
	.align	2
	.type	junk_fun15, %function
junk_fun15:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 12	// tmp110,
	str	w0, [sp, 8]	// tmp110, mod_reg_val
	mov	w0, 7	// tmp111,
	str	w0, [sp, 12]	// tmp111, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.84_1, mod_reg_val
	add	w0, w0, 1	// _2, mod_reg_val.84_1,
	str	w0, [sp, 8]	// _2, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.85_3, mod_reg_addr
	add	w0, w0, 51	// _4, mod_reg_addr.85_3,
	str	w0, [sp, 12]	// _4, mod_reg_addr
	ldr	w0, [sp, 12]	// mod_reg_addr.86_5, mod_reg_addr
	cmp	w0, 7	// mod_reg_addr.86_5,
	bne	.L69		//,
	ldr	w0, [sp, 12]	// _16, mod_reg_addr
	b	.L70		//
.L69:
	ldr	w1, [sp, 12]	// mod_reg_addr.87_6, mod_reg_addr
	mov	w0, 819	// tmp112,
	mul	w0, w1, w0	// _7, mod_reg_addr.87_6, tmp112
	str	w0, [sp, 8]	// _7, mod_reg_val
	ldr	w0, [sp, 8]	// mod_reg_val.88_8, mod_reg_val
	add	w0, w0, 1	// _9, mod_reg_val.88_8,
	str	w0, [sp, 8]	// _9, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.89_10, mod_reg_addr
	add	w0, w0, 51	// _11, mod_reg_addr.89_10,
	str	w0, [sp, 12]	// _11, mod_reg_addr
	ldr	w0, [sp, 12]	// mod_reg_addr.90_12, mod_reg_addr
	cmp	w0, 7	// mod_reg_addr.90_12,
	bne	.L71		//,
	ldr	w0, [sp, 12]	// _16, mod_reg_addr
	b	.L70		//
.L71:
	ldr	w1, [sp, 12]	// mod_reg_addr.91_26, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.92_13, mod_reg_val
	add	w0, w1, w0	// _14, mod_reg_addr.91_26, mod_reg_val.92_13
	str	w0, [sp, 8]	// _14, mod_reg_val
	ldr	w0, [sp, 8]	// mod_reg_val.93_15, mod_reg_val
	cmp	w0, 7	// mod_reg_val.93_15,
	bne	.L72		//,
	ldr	w0, [sp, 12]	// _16, mod_reg_addr
	b	.L70		//
.L72:
	ldr	w0, [sp, 8]	// _16, mod_reg_val
.L70:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun15, .-junk_fun15
	.align	2
	.type	junk_fun16, %function
junk_fun16:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 9	// tmp100,
	str	w0, [sp, 8]	// tmp100, mod_reg_val
	mov	w0, 6	// tmp101,
	str	w0, [sp, 12]	// tmp101, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.94_1, mod_reg_val
	add	w1, w0, 1	// _3, mod_reg_val.94_1,
	str	w1, [sp, 8]	// _3, mod_reg_val
	cmp	w0, 7	// mod_reg_val.94_1,
	bne	.L74		//,
	mov	w0, 51	// tmp102,
	str	w0, [sp, 12]	// tmp102, mod_reg_addr
.L74:
	ldr	w0, [sp, 8]	// mod_reg_val.96_4, mod_reg_val
	and	w0, w0, 2	// _5, mod_reg_val.96_4,
	str	w0, [sp, 12]	// _5, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.97_6, mod_reg_val
	cmp	w0, 16	// mod_reg_val.97_6,
	bne	.L75		//,
	ldr	w0, [sp, 12]	// _8, mod_reg_addr
	b	.L76		//
.L75:
	ldr	w0, [sp, 8]	// mod_reg_val.98_7, mod_reg_val
	cmp	w0, 7	// mod_reg_val.98_7,
	bne	.L77		//,
	ldr	w0, [sp, 12]	// _8, mod_reg_addr
	b	.L76		//
.L77:
	ldr	w0, [sp, 8]	// _8, mod_reg_val
.L76:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun16, .-junk_fun16
	.align	2
	.type	junk_fun17, %function
junk_fun17:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 14	// tmp101,
	str	w0, [sp, 8]	// tmp101, mod_reg_val
	mov	w0, 13	// tmp102,
	str	w0, [sp, 12]	// tmp102, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.99_1, mod_reg_val
	add	w1, w0, 1	// _3, mod_reg_val.99_1,
	str	w1, [sp, 8]	// _3, mod_reg_val
	and	w0, w0, 8	// _4, mod_reg_val.99_1,
	cmp	w0, 0	// _4,
	bne	.L79		//,
	ldr	w1, [sp, 8]	// mod_reg_val.101_14, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.102_5, mod_reg_addr
	mul	w0, w1, w0	// _6, mod_reg_val.101_14, mod_reg_addr.102_5
	str	w0, [sp, 12]	// _6, mod_reg_addr
.L79:
	ldr	w0, [sp, 8]	// mod_reg_val.103_7, mod_reg_val
	cmp	w0, 16	// mod_reg_val.103_7,
	bne	.L80		//,
	ldr	w0, [sp, 12]	// _8, mod_reg_addr
	b	.L81		//
.L80:
	ldr	w0, [sp, 8]	// _8, mod_reg_val
.L81:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun17, .-junk_fun17
	.align	2
	.type	junk_fun18, %function
junk_fun18:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 9	// tmp103,
	str	w0, [sp, 8]	// tmp103, mod_reg_val
	mov	w0, 15	// tmp104,
	str	w0, [sp, 12]	// tmp104, mod_reg_addr
	ldr	w1, [sp, 12]	// mod_reg_addr.104_13, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.105_1, mod_reg_val
	eor	w0, w1, w0	// _2, mod_reg_addr.104_13, mod_reg_val.105_1
	str	w0, [sp, 8]	// _2, mod_reg_val
	ldr	w0, [sp, 8]	// mod_reg_val.106_3, mod_reg_val
	add	w1, w0, 1	// _5, mod_reg_val.106_3,
	str	w1, [sp, 8]	// _5, mod_reg_val
	and	w0, w0, 8	// _6, mod_reg_val.106_3,
	cmp	w0, 0	// _6,
	bne	.L83		//,
	ldr	w1, [sp, 8]	// mod_reg_val.108_16, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.109_7, mod_reg_addr
	mul	w0, w1, w0	// _8, mod_reg_val.108_16, mod_reg_addr.109_7
	str	w0, [sp, 12]	// _8, mod_reg_addr
.L83:
	ldr	w0, [sp, 8]	// _18, mod_reg_val
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun18, .-junk_fun18
	.align	2
	.type	junk_fun19, %function
junk_fun19:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 1	// tmp101,
	str	w0, [sp, 8]	// tmp101, mod_reg_val
	mov	w0, 15	// tmp102,
	str	w0, [sp, 12]	// tmp102, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.110_1, mod_reg_val
	add	w1, w0, 1	// _3, mod_reg_val.110_1,
	str	w1, [sp, 8]	// _3, mod_reg_val
	and	w0, w0, 8	// _4, mod_reg_val.110_1,
	cmp	w0, 0	// _4,
	bne	.L86		//,
	ldr	w1, [sp, 8]	// mod_reg_val.112_14, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.113_5, mod_reg_addr
	mul	w0, w1, w0	// _6, mod_reg_val.112_14, mod_reg_addr.113_5
	str	w0, [sp, 12]	// _6, mod_reg_addr
.L86:
	ldr	w0, [sp, 8]	// mod_reg_val.114_7, mod_reg_val
	cmp	w0, 7	// mod_reg_val.114_7,
	bne	.L87		//,
	ldr	w0, [sp, 12]	// _8, mod_reg_addr
	b	.L88		//
.L87:
	ldr	w0, [sp, 8]	// _8, mod_reg_val
.L88:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun19, .-junk_fun19
	.align	2
	.type	junk_fun20, %function
junk_fun20:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 16	// tmp95,
	str	w0, [sp, 8]	// tmp95, mod_reg_val
	mov	w0, 8	// tmp96,
	str	w0, [sp, 12]	// tmp96, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.115_1, mod_reg_val
	cmp	w0, 7	// mod_reg_val.115_1,
	bne	.L90		//,
	ldr	w0, [sp, 12]	// _2, mod_reg_addr
	b	.L91		//
.L90:
	ldr	w0, [sp, 8]	// _2, mod_reg_val
.L91:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun20, .-junk_fun20
	.align	2
	.type	junk_fun21, %function
junk_fun21:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 1	// tmp104,
	str	w0, [sp, 8]	// tmp104, mod_reg_val
	mov	w0, 5	// tmp105,
	str	w0, [sp, 12]	// tmp105, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.116_1, mod_reg_val
	cmp	w0, 819	// mod_reg_val.116_1,
	bne	.L93		//,
	ldr	w0, [sp, 12]	// _11, mod_reg_addr
	b	.L94		//
.L93:
	ldr	w0, [sp, 12]	// mod_reg_addr.117_2, mod_reg_addr
	cmp	w0, 16	// mod_reg_addr.117_2,
	bne	.L95		//,
	ldr	w0, [sp, 8]	// _11, mod_reg_val
	b	.L94		//
.L95:
	ldr	w0, [sp, 8]	// mod_reg_val.118_3, mod_reg_val
	add	w1, w0, 1	// _5, mod_reg_val.118_3,
	str	w1, [sp, 8]	// _5, mod_reg_val
	and	w0, w0, 8	// _6, mod_reg_val.118_3,
	cmp	w0, 0	// _6,
	bne	.L96		//,
	ldr	w1, [sp, 8]	// mod_reg_val.120_18, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.121_7, mod_reg_addr
	mul	w0, w1, w0	// _8, mod_reg_val.120_18, mod_reg_addr.121_7
	str	w0, [sp, 12]	// _8, mod_reg_addr
.L96:
	ldr	w0, [sp, 8]	// mod_reg_val.122_9, mod_reg_val
	cmp	w0, 819	// mod_reg_val.122_9,
	bne	.L97		//,
	ldr	w0, [sp, 12]	// _11, mod_reg_addr
	b	.L94		//
.L97:
	ldr	w0, [sp, 8]	// mod_reg_val.123_10, mod_reg_val
	cmp	w0, 7	// mod_reg_val.123_10,
	bne	.L98		//,
	ldr	w0, [sp, 12]	// _11, mod_reg_addr
	b	.L94		//
.L98:
	ldr	w0, [sp, 8]	// _11, mod_reg_val
.L94:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun21, .-junk_fun21
	.align	2
	.type	junk_fun22, %function
junk_fun22:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 1	// tmp102,
	str	w0, [sp, 8]	// tmp102, mod_reg_val
	mov	w0, 10	// tmp103,
	str	w0, [sp, 12]	// tmp103, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.124_1, mod_reg_val
	add	w0, w0, 1	// _2, mod_reg_val.124_1,
	str	w0, [sp, 8]	// _2, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.125_3, mod_reg_addr
	add	w0, w0, 51	// _4, mod_reg_addr.125_3,
	str	w0, [sp, 12]	// _4, mod_reg_addr
	ldr	w0, [sp, 12]	// mod_reg_addr.126_5, mod_reg_addr
	cmp	w0, 7	// mod_reg_addr.126_5,
	bne	.L100		//,
	ldr	w0, [sp, 12]	// _8, mod_reg_addr
	b	.L101		//
.L100:
	ldr	w1, [sp, 12]	// mod_reg_addr.127_15, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.128_6, mod_reg_val
	eor	w0, w1, w0	// _7, mod_reg_addr.127_15, mod_reg_val.128_6
	str	w0, [sp, 8]	// _7, mod_reg_val
	ldr	w0, [sp, 8]	// _8, mod_reg_val
.L101:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun22, .-junk_fun22
	.align	2
	.type	junk_fun23, %function
junk_fun23:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 13	// tmp99,
	str	w0, [sp, 8]	// tmp99, mod_reg_val
	mov	w0, 9	// tmp100,
	str	w0, [sp, 12]	// tmp100, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.129_1, mod_reg_val
	cmp	w0, 7	// mod_reg_val.129_1,
	bne	.L103		//,
	ldr	w0, [sp, 12]	// _5, mod_reg_addr
	b	.L104		//
.L103:
	ldr	w0, [sp, 8]	// mod_reg_val.130_2, mod_reg_val
	cmp	w0, 16	// mod_reg_val.130_2,
	bne	.L105		//,
	ldr	w0, [sp, 12]	// _5, mod_reg_addr
	b	.L104		//
.L105:
	ldr	w1, [sp, 8]	// mod_reg_val.131_10, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.132_3, mod_reg_addr
	eor	w0, w1, w0	// _4, mod_reg_val.131_10, mod_reg_addr.132_3
	str	w0, [sp, 12]	// _4, mod_reg_addr
	ldr	w0, [sp, 8]	// _5, mod_reg_val
.L104:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun23, .-junk_fun23
	.align	2
	.type	junk_fun24, %function
junk_fun24:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 12	// tmp101,
	str	w0, [sp, 8]	// tmp101, mod_reg_val
	mov	w0, 13	// tmp102,
	str	w0, [sp, 12]	// tmp102, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.133_1, mod_reg_val
	add	w0, w0, 1	// _2, mod_reg_val.133_1,
	str	w0, [sp, 8]	// _2, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.134_3, mod_reg_addr
	add	w0, w0, 51	// _4, mod_reg_addr.134_3,
	str	w0, [sp, 12]	// _4, mod_reg_addr
	ldr	w0, [sp, 12]	// mod_reg_addr.135_5, mod_reg_addr
	cmp	w0, 7	// mod_reg_addr.135_5,
	bne	.L107		//,
	ldr	w0, [sp, 12]	// _8, mod_reg_addr
	b	.L108		//
.L107:
	ldr	w1, [sp, 8]	// mod_reg_val.136_6, mod_reg_val
	mov	w0, 243	// tmp103,
	and	w0, w1, w0	// _7, mod_reg_val.136_6, tmp103
	str	w0, [sp, 12]	// _7, mod_reg_addr
	ldr	w0, [sp, 8]	// _8, mod_reg_val
.L108:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun24, .-junk_fun24
	.align	2
	.type	junk_fun25, %function
junk_fun25:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 4	// tmp104,
	str	w0, [sp, 8]	// tmp104, mod_reg_val
	mov	w0, 11	// tmp105,
	str	w0, [sp, 12]	// tmp105, mod_reg_addr
	ldr	w0, [sp, 12]	// mod_reg_addr.137_1, mod_reg_addr
	and	w0, w0, 1	// _2, mod_reg_addr.137_1,
	cmp	w0, 768	// _2,
	beq	.L110		//,
	ldr	w1, [sp, 8]	// mod_reg_val.138_14, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.139_3, mod_reg_addr
	add	w0, w1, w0	// _4, mod_reg_val.138_14, mod_reg_addr.139_3
	str	w0, [sp, 12]	// _4, mod_reg_addr
.L110:
	ldr	w1, [sp, 12]	// mod_reg_addr.140_16, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.141_5, mod_reg_val
	add	w0, w1, w0	// _6, mod_reg_addr.140_16, mod_reg_val.141_5
	str	w0, [sp, 8]	// _6, mod_reg_val
	ldr	w1, [sp, 8]	// mod_reg_val.142_7, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.143_8, mod_reg_addr
	cmp	w1, w0	// mod_reg_val.142_7, mod_reg_addr.143_8
	bge	.L111		//,
	ldr	w0, [sp, 12]	// _9, mod_reg_addr
	b	.L112		//
.L111:
	ldr	w0, [sp, 8]	// _9, mod_reg_val
.L112:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun25, .-junk_fun25
	.align	2
	.type	junk_fun26, %function
junk_fun26:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 5	// tmp104,
	str	w0, [sp, 8]	// tmp104, mod_reg_val
	mov	w0, 8	// tmp105,
	str	w0, [sp, 12]	// tmp105, mod_reg_addr
	ldr	w1, [sp, 12]	// mod_reg_addr.144_16, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.145_1, mod_reg_val
	eor	w0, w1, w0	// _2, mod_reg_addr.144_16, mod_reg_val.145_1
	str	w0, [sp, 8]	// _2, mod_reg_val
	ldr	w1, [sp, 8]	// mod_reg_val.146_3, mod_reg_val
	mov	w0, 243	// tmp106,
	and	w0, w1, w0	// _4, mod_reg_val.146_3, tmp106
	str	w0, [sp, 12]	// _4, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.147_5, mod_reg_val
	add	w1, w0, 1	// _7, mod_reg_val.147_5,
	str	w1, [sp, 8]	// _7, mod_reg_val
	cmp	w0, 7	// mod_reg_val.147_5,
	bne	.L114		//,
	mov	w0, 51	// tmp107,
	str	w0, [sp, 12]	// tmp107, mod_reg_addr
.L114:
	ldr	w0, [sp, 8]	// mod_reg_val.149_8, mod_reg_val
	cmp	w0, 7	// mod_reg_val.149_8,
	bne	.L115		//,
	ldr	w0, [sp, 12]	// _11, mod_reg_addr
	b	.L116		//
.L115:
	ldr	w1, [sp, 8]	// mod_reg_val.150_9, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.151_10, mod_reg_addr
	cmp	w1, w0	// mod_reg_val.150_9, mod_reg_addr.151_10
	bge	.L117		//,
	ldr	w0, [sp, 12]	// _11, mod_reg_addr
	b	.L116		//
.L117:
	ldr	w0, [sp, 8]	// _11, mod_reg_val
.L116:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun26, .-junk_fun26
	.align	2
	.type	junk_fun27, %function
junk_fun27:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 5	// tmp103,
	str	w0, [sp, 8]	// tmp103, mod_reg_val
	mov	w0, 4	// tmp104,
	str	w0, [sp, 12]	// tmp104, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.152_1, mod_reg_val
	cmp	w0, 16	// mod_reg_val.152_1,
	bne	.L119		//,
	ldr	w0, [sp, 12]	// _8, mod_reg_addr
	b	.L120		//
.L119:
	ldr	w1, [sp, 8]	// mod_reg_val.153_13, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.154_2, mod_reg_addr
	eor	w0, w1, w0	// _3, mod_reg_val.153_13, mod_reg_addr.154_2
	str	w0, [sp, 12]	// _3, mod_reg_addr
	ldr	w1, [sp, 8]	// mod_reg_val.155_4, mod_reg_val
	mov	w0, 243	// tmp105,
	and	w0, w1, w0	// _5, mod_reg_val.155_4, tmp105
	str	w0, [sp, 12]	// _5, mod_reg_addr
	ldr	w1, [sp, 8]	// mod_reg_val.156_16, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.157_6, mod_reg_addr
	eor	w0, w1, w0	// _7, mod_reg_val.156_16, mod_reg_addr.157_6
	str	w0, [sp, 12]	// _7, mod_reg_addr
	ldr	w0, [sp, 8]	// _8, mod_reg_val
.L120:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun27, .-junk_fun27
	.align	2
	.type	junk_fun28, %function
junk_fun28:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 3	// tmp104,
	str	w0, [sp, 8]	// tmp104, mod_reg_val
	mov	w0, 13	// tmp105,
	str	w0, [sp, 12]	// tmp105, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.158_1, mod_reg_val
	cmp	w0, 819	// mod_reg_val.158_1,
	bne	.L122		//,
	ldr	w0, [sp, 12]	// _10, mod_reg_addr
	b	.L123		//
.L122:
	ldr	w1, [sp, 8]	// mod_reg_val.159_15, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.160_2, mod_reg_addr
	eor	w0, w1, w0	// _3, mod_reg_val.159_15, mod_reg_addr.160_2
	str	w0, [sp, 12]	// _3, mod_reg_addr
	ldr	w1, [sp, 8]	// mod_reg_val.161_4, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.162_5, mod_reg_addr
	cmp	w1, w0	// mod_reg_val.161_4, mod_reg_addr.162_5
	bge	.L124		//,
	ldr	w0, [sp, 12]	// _10, mod_reg_addr
	b	.L123		//
.L124:
	ldr	w1, [sp, 8]	// mod_reg_val.163_6, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.164_7, mod_reg_addr
	cmp	w1, w0	// mod_reg_val.163_6, mod_reg_addr.164_7
	bge	.L125		//,
	ldr	w0, [sp, 12]	// _10, mod_reg_addr
	b	.L123		//
.L125:
	ldr	w0, [sp, 8]	// mod_reg_val.165_8, mod_reg_val
	and	w0, w0, 2	// _9, mod_reg_val.165_8,
	str	w0, [sp, 12]	// _9, mod_reg_addr
	ldr	w0, [sp, 8]	// _10, mod_reg_val
.L123:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun28, .-junk_fun28
	.align	2
	.type	junk_fun29, %function
junk_fun29:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 4	// tmp108,
	str	w0, [sp, 8]	// tmp108, mod_reg_val
	mov	w0, 4	// tmp109,
	str	w0, [sp, 12]	// tmp109, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.166_1, mod_reg_val
	cmp	w0, 819	// mod_reg_val.166_1,
	bne	.L127		//,
	ldr	w0, [sp, 12]	// _15, mod_reg_addr
	b	.L128		//
.L127:
	ldr	w0, [sp, 8]	// mod_reg_val.167_2, mod_reg_val
	cmp	w0, 819	// mod_reg_val.167_2,
	bne	.L129		//,
	ldr	w0, [sp, 12]	// _15, mod_reg_addr
	b	.L128		//
.L129:
	ldr	w0, [sp, 8]	// mod_reg_val.168_3, mod_reg_val
	add	w1, w0, 1	// _5, mod_reg_val.168_3,
	str	w1, [sp, 8]	// _5, mod_reg_val
	cmp	w0, 7	// mod_reg_val.168_3,
	bne	.L130		//,
	mov	w0, 51	// tmp110,
	str	w0, [sp, 12]	// tmp110, mod_reg_addr
.L130:
	ldr	w0, [sp, 12]	// mod_reg_addr.170_6, mod_reg_addr
	and	w0, w0, 1	// _7, mod_reg_addr.170_6,
	cmp	w0, 768	// _7,
	beq	.L131		//,
	ldr	w1, [sp, 8]	// mod_reg_val.171_24, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.172_8, mod_reg_addr
	add	w0, w1, w0	// _9, mod_reg_val.171_24, mod_reg_addr.172_8
	str	w0, [sp, 12]	// _9, mod_reg_addr
.L131:
	ldr	w0, [sp, 8]	// mod_reg_val.173_10, mod_reg_val
	add	w0, w0, 1	// _11, mod_reg_val.173_10,
	str	w0, [sp, 8]	// _11, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.174_12, mod_reg_addr
	add	w0, w0, 51	// _13, mod_reg_addr.174_12,
	str	w0, [sp, 12]	// _13, mod_reg_addr
	ldr	w0, [sp, 12]	// mod_reg_addr.175_14, mod_reg_addr
	cmp	w0, 7	// mod_reg_addr.175_14,
	bne	.L132		//,
	ldr	w0, [sp, 12]	// _15, mod_reg_addr
	b	.L128		//
.L132:
	ldr	w0, [sp, 8]	// _15, mod_reg_val
.L128:
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun29, .-junk_fun29
	.align	2
	.type	junk_fun30, %function
junk_fun30:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 15	// tmp107,
	str	w0, [sp, 8]	// tmp107, mod_reg_val
	mov	w0, 7	// tmp108,
	str	w0, [sp, 12]	// tmp108, mod_reg_addr
	ldr	w1, [sp, 12]	// mod_reg_addr.176_1, mod_reg_addr
	mov	w0, 819	// tmp109,
	mul	w0, w1, w0	// _2, mod_reg_addr.176_1, tmp109
	str	w0, [sp, 8]	// _2, mod_reg_val
	ldr	w1, [sp, 12]	// mod_reg_addr.177_16, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.178_3, mod_reg_val
	add	w0, w1, w0	// _4, mod_reg_addr.177_16, mod_reg_val.178_3
	str	w0, [sp, 8]	// _4, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.179_5, mod_reg_addr
	and	w0, w0, 1	// _6, mod_reg_addr.179_5,
	cmp	w0, 768	// _6,
	beq	.L134		//,
	ldr	w1, [sp, 8]	// mod_reg_val.180_18, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.181_7, mod_reg_addr
	add	w0, w1, w0	// _8, mod_reg_val.180_18, mod_reg_addr.181_7
	str	w0, [sp, 12]	// _8, mod_reg_addr
.L134:
	ldr	w1, [sp, 12]	// mod_reg_addr.182_20, mod_reg_addr
	ldr	w0, [sp, 8]	// mod_reg_val.183_9, mod_reg_val
	add	w0, w1, w0	// _10, mod_reg_addr.182_20, mod_reg_val.183_9
	str	w0, [sp, 8]	// _10, mod_reg_val
	ldr	w0, [sp, 8]	// _22, mod_reg_val
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	junk_fun30, .-junk_fun30
	.align	2
	.type	fun_a10, %function
fun_a10:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 512	// tmp100,
	str	w0, [sp, 8]	// tmp100, mod_reg_val
	mov	w0, 50	// tmp101,
	str	w0, [sp, 12]	// tmp101, mod_reg_addr
	ldr	w0, [sp, 12]	// mod_reg_addr.184_1, mod_reg_addr
	add	w0, w0, 100	// _2, mod_reg_addr.184_1,
	str	w0, [sp, 12]	// _2, mod_reg_addr
	ldr	w0, [sp, 12]	// mod_reg_addr.185_3, mod_reg_addr
	sub	w0, w0, #100	// _4, mod_reg_addr.185_3,
	str	w0, [sp, 12]	// _4, mod_reg_addr
	ldr	w1, [sp, 8]	// mod_reg_val.186_5, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.187_6, mod_reg_addr
	add	w0, w1, w0	// _12, mod_reg_val.186_5, mod_reg_addr.187_6
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	fun_a10, .-fun_a10
	.align	2
	.type	fun_a11, %function
fun_a11:
	hint	25 // paciasp
	sub	sp, sp, #16	//,,
	mov	w0, 256	// tmp96,
	str	w0, [sp, 8]	// tmp96, mod_reg_val
	mov	w0, 66	// tmp97,
	str	w0, [sp, 12]	// tmp97, mod_reg_addr
	ldr	w1, [sp, 8]	// mod_reg_val.188_1, mod_reg_val
	ldr	w0, [sp, 12]	// mod_reg_addr.189_2, mod_reg_addr
	add	w0, w1, w0	// _6, mod_reg_val.188_1, mod_reg_addr.189_2
	add	sp, sp, 16	//,,
	hint	29 // autiasp
	ret	
	.size	fun_a11, .-fun_a11
	.global	res
	.bss
	.align	3
	.type	res, %object
	.size	res, 32
res:
	.zero	32
	.text
	.align	2
	.global	awlink_asm_write_cmdreg
	.type	awlink_asm_write_cmdreg, %function
awlink_asm_write_cmdreg:
	hint	25 // paciasp
	stp	x29, x30, [sp, -48]!	//,,,
	mov	x29, sp	//,
	str	w0, [sp, 44]	// mod_reg_val, mod_reg_val
	str	x1, [sp, 32]	// mod_reg_addr, mod_reg_addr
	str	x2, [sp, 24]	// reg_addr, reg_addr
	bl	junk_fun1		//
	ldr	x0, [sp, 32]	// tmp93, mod_reg_addr
	add	x0, x0, 4	// _1, tmp93,
	mov	x1, x0	//, _1
	ldr	w0, [sp, 44]	//, mod_reg_val
	bl	writel		//
	bl	junk_fun2		//
	nop	
	ldp	x29, x30, [sp], 48	//,,,
	hint	29 // autiasp
	ret	
	.size	awlink_asm_write_cmdreg, .-awlink_asm_write_cmdreg
	.align	2
	.global	awlink_asm_fun0
	.type	awlink_asm_fun0, %function
awlink_asm_fun0:
	hint	25 // paciasp
	sub	sp, sp, #48	//,,
	stp	x29, x30, [sp, 16]	//,,
	add	x29, sp, 16	//,,
	str	x0, [sp, 40]	// num, num
	str	x1, [sp, 32]	// pin_id, pin_id
	ldr	x1, [sp, 32]	// pin_id.190_1, pin_id
	str	xzr, [sp, 8]	//,
	adrp	x0, res	// tmp94,
	add	x0, x0, :lo12:res	// tmp93, tmp94,
	str	x0, [sp]	// tmp93,
	mov	x7, 0	//,
	mov	x6, 0	//,
	mov	x5, 0	//,
	mov	x4, x1	//, pin_id.190_1
	mov	x3, 0	//,
	ldr	x2, [sp, 40]	//, num
	mov	w1, 4294901767	//,
	mov	x0, 160	//,
	movk	x0, 0xc000, lsl 16	//,,
	bl	__arm_smccc_smc		//
	nop	
	ldp	x29, x30, [sp, 16]	//,,
	add	sp, sp, 48	//,,
	hint	29 // autiasp
	ret	
	.size	awlink_asm_fun0, .-awlink_asm_fun0
	.align	2
	.global	awlink_asm_fun1
	.type	awlink_asm_fun1, %function
awlink_asm_fun1:
	hint	25 // paciasp
	sub	sp, sp, #48	//,,
	stp	x29, x30, [sp, 16]	//,,
	add	x29, sp, 16	//,,
	str	x0, [sp, 40]	// num, num
	str	xzr, [sp, 8]	//,
	adrp	x0, res	// tmp93,
	add	x0, x0, :lo12:res	// tmp92, tmp93,
	str	x0, [sp]	// tmp92,
	mov	x7, 0	//,
	mov	x6, 0	//,
	mov	x5, 0	//,
	mov	x4, 0	//,
	mov	x3, 0	//,
	ldr	x2, [sp, 40]	//, num
	mov	w1, 4294901763	//,
	mov	x0, 160	//,
	movk	x0, 0xc000, lsl 16	//,,
	bl	__arm_smccc_smc		//
	nop	
	ldp	x29, x30, [sp, 16]	//,,
	add	sp, sp, 48	//,,
	hint	29 // autiasp
	ret	
	.size	awlink_asm_fun1, .-awlink_asm_fun1
	.align	2
	.global	awlink_asm_fun2
	.type	awlink_asm_fun2, %function
awlink_asm_fun2:
	hint	25 // paciasp
	sub	sp, sp, #48	//,,
	stp	x29, x30, [sp, 16]	//,,
	add	x29, sp, 16	//,,
	str	x0, [sp, 40]	// num, num
	str	xzr, [sp, 8]	//,
	adrp	x0, res	// tmp93,
	add	x0, x0, :lo12:res	// tmp92, tmp93,
	str	x0, [sp]	// tmp92,
	mov	x7, 0	//,
	mov	x6, 0	//,
	mov	x5, 0	//,
	mov	x4, 0	//,
	mov	x3, 0	//,
	ldr	x2, [sp, 40]	//, num
	mov	w1, 4294901764	//,
	mov	x0, 160	//,
	movk	x0, 0xc000, lsl 16	//,,
	bl	__arm_smccc_smc		//
	nop	
	ldp	x29, x30, [sp, 16]	//,,
	add	sp, sp, 48	//,,
	hint	29 // autiasp
	ret	
	.size	awlink_asm_fun2, .-awlink_asm_fun2
	.align	2
	.global	awlink_asm_fun3
	.type	awlink_asm_fun3, %function
awlink_asm_fun3:
	hint	25 // paciasp
	sub	sp, sp, #48	//,,
	stp	x29, x30, [sp, 16]	//,,
	add	x29, sp, 16	//,,
	str	x0, [sp, 40]	// num, num
	str	xzr, [sp, 8]	//,
	adrp	x0, res	// tmp93,
	add	x0, x0, :lo12:res	// tmp92, tmp93,
	str	x0, [sp]	// tmp92,
	mov	x7, 0	//,
	mov	x6, 0	//,
	mov	x5, 0	//,
	mov	x4, 0	//,
	mov	x3, 0	//,
	mov	x2, 0	//,
	mov	w1, 4294901765	//,
	mov	x0, 160	//,
	movk	x0, 0xc000, lsl 16	//,,
	bl	__arm_smccc_smc		//
	nop	
	ldp	x29, x30, [sp, 16]	//,,
	add	sp, sp, 48	//,,
	hint	29 // autiasp
	ret	
	.size	awlink_asm_fun3, .-awlink_asm_fun3
	.align	2
	.global	awlink_asm_fun4
	.type	awlink_asm_fun4, %function
awlink_asm_fun4:
	hint	25 // paciasp
	sub	sp, sp, #48	//,,
	stp	x29, x30, [sp, 16]	//,,
	add	x29, sp, 16	//,,
	str	x0, [sp, 40]	// num, num
	str	xzr, [sp, 8]	//,
	adrp	x0, res	// tmp93,
	add	x0, x0, :lo12:res	// tmp92, tmp93,
	str	x0, [sp]	// tmp92,
	mov	x7, 0	//,
	mov	x6, 0	//,
	mov	x5, 0	//,
	mov	x4, 0	//,
	mov	x3, 0	//,
	mov	x2, 0	//,
	mov	w1, 4294901766	//,
	mov	x0, 160	//,
	movk	x0, 0xc000, lsl 16	//,,
	bl	__arm_smccc_smc		//
	nop	
	ldp	x29, x30, [sp, 16]	//,,
	add	sp, sp, 48	//,,
	hint	29 // autiasp
	ret	
	.size	awlink_asm_fun4, .-awlink_asm_fun4
	.align	2
	.global	awlink_asm_fun5
	.type	awlink_asm_fun5, %function
awlink_asm_fun5:
	hint	25 // paciasp
	sub	sp, sp, #48	//,,
	stp	x29, x30, [sp, 16]	//,,
	add	x29, sp, 16	//,,
	str	x0, [sp, 40]	// num, num
	str	xzr, [sp, 8]	//,
	adrp	x0, res	// tmp100,
	add	x0, x0, :lo12:res	// tmp99, tmp100,
	str	x0, [sp]	// tmp99,
	mov	x7, 0	//,
	mov	x6, 0	//,
	mov	x5, 0	//,
	mov	x4, 0	//,
	mov	x3, 0	//,
	ldr	x2, [sp, 40]	//, num
	mov	w1, 4294901768	//,
	mov	x0, 160	//,
	movk	x0, 0xc000, lsl 16	//,,
	bl	__arm_smccc_smc		//
	adrp	x0, res	// tmp102,
	add	x0, x0, :lo12:res	// tmp101, tmp102,
	ldr	x3, [x0]	// _1, res.a0
	adrp	x0, res	// tmp104,
	add	x0, x0, :lo12:res	// tmp103, tmp104,
	ldr	x1, [x0, 8]	// _2, res.a1
	adrp	x0, arm64_use_ng_mappings	// tmp106,
	add	x0, x0, :lo12:arm64_use_ng_mappings	// tmp105, tmp106,
	ldrb	w0, [x0]	// arm64_use_ng_mappings.192_3, arm64_use_ng_mappings
	cmp	w0, 0	// arm64_use_ng_mappings.192_3,
	beq	.L147		//,
	mov	x0, 3859	// iftmp.191_4,
	movk	x0, 0x68, lsl 48	// iftmp.191_4,,
	b	.L148		//
.L147:
	mov	x0, 1811	// iftmp.191_4,
	movk	x0, 0x68, lsl 48	// iftmp.191_4,,
.L148:
	mov	x2, x0	//, D.69538
	mov	x0, x3	//, _1
	bl	__ioremap		//
	ldp	x29, x30, [sp, 16]	//,,
	add	sp, sp, 48	//,,
	hint	29 // autiasp
	ret	
	.size	awlink_asm_fun5, .-awlink_asm_fun5
	.align	2
	.global	awlink_asm_start
	.type	awlink_asm_start, %function
awlink_asm_start:
	hint	25 // paciasp
	sub	sp, sp, #64	//,,
	stp	x29, x30, [sp, 16]	//,,
	add	x29, sp, 16	//,,
	str	x0, [sp, 56]	// mod_reg_addr, mod_reg_addr
	str	x1, [sp, 48]	// ctrlmode, ctrlmode
	str	x2, [sp, 40]	// num, num
	str	xzr, [sp, 8]	//,
	adrp	x0, res	// tmp93,
	add	x0, x0, :lo12:res	// tmp92, tmp93,
	str	x0, [sp]	// tmp92,
	mov	x7, 0	//,
	mov	x6, 0	//,
	mov	x5, 0	//,
	mov	x4, 0	//,
	ldr	x3, [sp, 48]	//, ctrlmode
	ldr	x2, [sp, 40]	//, num
	mov	w1, 4294901761	//,
	mov	x0, 160	//,
	movk	x0, 0xc000, lsl 16	//,,
	bl	__arm_smccc_smc		//
	bl	junk_fun4		//
	nop	
	ldp	x29, x30, [sp, 16]	//,,
	add	sp, sp, 64	//,,
	hint	29 // autiasp
	ret	
	.size	awlink_asm_start, .-awlink_asm_start
	.align	2
	.global	awlink_asm_set_bittiming
	.type	awlink_asm_set_bittiming, %function
awlink_asm_set_bittiming:
	hint	25 // paciasp
	stp	x29, x30, [sp, -48]!	//,,,
	mov	x29, sp	//,
	str	x0, [sp, 40]	// mod_reg_addr, mod_reg_addr
	str	w1, [sp, 36]	// mod_reg_val, mod_reg_val
	str	x2, [sp, 24]	// cfg, cfg
	str	x3, [sp, 16]	// reg_addr, reg_addr
	bl	junk_fun9		//
	ldr	w0, [sp, 36]	// tmp97, mod_reg_val
	and	w0, w0, 4	// _1, tmp97,
	cmp	w0, 0	// _1,
	beq	.L152		//,
	ldr	x0, [sp, 24]	// tmp98, cfg
	ldr	w0, [x0]	// _2, *cfg_10(D)
	orr	w1, w0, 8388608	// _3, _2,
	ldr	x0, [sp, 24]	// tmp99, cfg
	str	w1, [x0]	// _3, *cfg_10(D)
.L152:
	bl	junk_fun10		//
	ldr	x0, [sp, 24]	// tmp100, cfg
	ldr	w2, [x0]	// _4, *cfg_10(D)
	ldr	x0, [sp, 40]	// tmp101, mod_reg_addr
	add	x0, x0, 20	// _5, tmp101,
	mov	x1, x0	//, _5
	mov	w0, w2	//, _4
	bl	writel		//
	nop	
	ldp	x29, x30, [sp], 48	//,,,
	hint	29 // autiasp
	ret	
	.size	awlink_asm_set_bittiming, .-awlink_asm_set_bittiming
	.align	2
	.global	awlink_asm_clean_transfer_err
	.type	awlink_asm_clean_transfer_err, %function
awlink_asm_clean_transfer_err:
	hint	25 // paciasp
	stp	x29, x30, [sp, -64]!	//,,,
	mov	x29, sp	//,
	str	x0, [sp, 40]	// mod_reg_addr, mod_reg_addr
	str	x1, [sp, 32]	// t_err, t_err
	str	x2, [sp, 24]	// r_err, r_err
	bl	junk_fun12		//
	ldr	x0, [sp, 40]	// tmp98, mod_reg_addr
	add	x0, x0, 28	// _1, tmp98,
	bl	readl		//
	str	w0, [sp, 60]	//, errors
	bl	junk_fun13		//
	ldr	w0, [sp, 60]	// tmp99, errors
	and	w0, w0, 65535	// _2, tmp99
	and	w0, w0, 255	// tmp100, _2,
	and	w1, w0, 65535	// _3, tmp100
	ldr	x0, [sp, 32]	// tmp101, t_err
	strh	w1, [x0]	// tmp102, *t_err_13(D)
	ldr	w0, [sp, 60]	// tmp103, errors
	lsr	w0, w0, 16	// _4, tmp103,
	and	w0, w0, 65535	// _5, _4
	and	w0, w0, 255	// tmp104, _5,
	and	w1, w0, 65535	// _6, tmp104
	ldr	x0, [sp, 24]	// tmp105, r_err
	strh	w1, [x0]	// tmp106, *r_err_15(D)
	nop	
	ldp	x29, x30, [sp], 64	//,,,
	hint	29 // autiasp
	ret	
	.size	awlink_asm_clean_transfer_err, .-awlink_asm_clean_transfer_err
	.align	2
	.global	awlink_asm_start_xmit
	.type	awlink_asm_start_xmit, %function
awlink_asm_start_xmit:
	hint	25 // paciasp
	stp	x29, x30, [sp, -80]!	//,,,
	mov	x29, sp	//,
	str	x0, [sp, 56]	// mod_reg_addr, mod_reg_addr
	str	x1, [sp, 48]	// addr, addr
	str	w2, [sp, 44]	// id, id
	str	x3, [sp, 32]	// flag, flag
	str	x4, [sp, 24]	// mod_reg_val, mod_reg_val
	str	x5, [sp, 16]	// pdata, pdata
	str	wzr, [sp, 72]	//, i
	ldr	x0, [sp, 32]	// tmp137, flag
	ldr	w0, [x0]	// tmp138, *flag_52(D)
	str	w0, [sp, 76]	// tmp138, dlc
	ldr	w0, [sp, 44]	// id.193_1, id
	and	w0, w0, 1073741824	// _2, id.193_1,
	cmp	w0, 0	// _2,
	beq	.L155		//,
	ldr	x0, [sp, 32]	// tmp139, flag
	ldr	w0, [x0]	// _3, *flag_52(D)
	orr	w1, w0, 64	// _4, _3,
	ldr	x0, [sp, 32]	// tmp140, flag
	str	w1, [x0]	// _4, *flag_52(D)
.L155:
	ldr	w0, [sp, 44]	// id.194_5, id
	cmp	w0, 0	// id.195_6,
	bge	.L156		//,
	ldr	x0, [sp, 32]	// tmp141, flag
	ldr	w0, [x0]	// _7, *flag_52(D)
	orr	w1, w0, 128	// _8, _7,
	ldr	x0, [sp, 32]	// tmp142, flag
	str	w1, [x0]	// _8, *flag_52(D)
	ldr	x0, [sp, 24]	// tmp143, mod_reg_val
	mov	w1, 84	// tmp144,
	str	w1, [x0]	// tmp144, *mod_reg_val_55(D)
	ldr	w0, [sp, 44]	// id.196_9, id
	lsr	w0, w0, 21	// _10, id.196_9,
	and	w2, w0, 255	// _11, _10,
	ldr	x0, [sp, 56]	// tmp145, mod_reg_addr
	add	x0, x0, 68	// _12, tmp145,
	mov	x1, x0	//, _12
	mov	w0, w2	//, _11
	bl	writel		//
	bl	junk_fun14		//
	ldr	w0, [sp, 44]	// id.197_13, id
	lsr	w0, w0, 13	// _14, id.197_13,
	and	w2, w0, 255	// _15, _14,
	ldr	x0, [sp, 56]	// tmp146, mod_reg_addr
	add	x0, x0, 72	// _16, tmp146,
	mov	x1, x0	//, _16
	mov	w0, w2	//, _15
	bl	writel		//
	ldr	w0, [sp, 44]	// id.198_17, id
	lsr	w0, w0, 5	// _18, id.198_17,
	and	w2, w0, 255	// _19, _18,
	ldr	x0, [sp, 56]	// tmp147, mod_reg_addr
	add	x0, x0, 76	// _20, tmp147,
	mov	x1, x0	//, _20
	mov	w0, w2	//, _19
	bl	writel		//
	ldr	w0, [sp, 44]	// id.199_21, id
	lsl	w0, w0, 3	// _22, id.199_21,
	and	w2, w0, 248	// _23, _22,
	ldr	x0, [sp, 56]	// tmp148, mod_reg_addr
	add	x0, x0, 80	// _24, tmp148,
	mov	x1, x0	//, _24
	mov	w0, w2	//, _23
	bl	writel		//
	b	.L157		//
.L156:
	ldr	x0, [sp, 24]	// tmp149, mod_reg_val
	mov	w1, 76	// tmp150,
	str	w1, [x0]	// tmp150, *mod_reg_val_55(D)
	ldr	w0, [sp, 44]	// id.200_25, id
	lsr	w0, w0, 3	// _26, id.200_25,
	and	w2, w0, 255	// _27, _26,
	ldr	x0, [sp, 56]	// tmp151, mod_reg_addr
	add	x0, x0, 68	// _28, tmp151,
	mov	x1, x0	//, _28
	mov	w0, w2	//, _27
	bl	writel		//
	ldr	w0, [sp, 44]	// id.201_29, id
	lsl	w0, w0, 5	// _30, id.201_29,
	and	w2, w0, 224	// _31, _30,
	ldr	x0, [sp, 56]	// tmp152, mod_reg_addr
	add	x0, x0, 72	// _32, tmp152,
	mov	x1, x0	//, _32
	mov	w0, w2	//, _31
	bl	writel		//
.L157:
	str	wzr, [sp, 72]	//, i
	b	.L158		//
.L159:
	ldrsw	x0, [sp, 72]	// _33, i
	ldr	x1, [sp, 16]	// tmp153, pdata
	add	x0, x1, x0	// _34, tmp153, _33
	ldrb	w0, [x0]	// tmp154, *_34
	and	w0, w0, 255	// _35, tmp154
	mov	w2, w0	// _36, _35
	ldr	x0, [sp, 24]	// tmp155, mod_reg_val
	ldr	w0, [x0]	// _37, *mod_reg_val_55(D)
	ldr	w1, [sp, 72]	// tmp156, i
	lsl	w1, w1, 2	// _38, tmp156,
	add	w0, w0, w1	// _40, _37, _39
	uxtw	x0, w0	// _41, _40
	ldr	x1, [sp, 56]	// tmp157, mod_reg_addr
	add	x0, x1, x0	// _42, tmp157, _41
	mov	x1, x0	//, _42
	mov	w0, w2	//, _36
	bl	writel		//
	ldr	w0, [sp, 72]	// tmp159, i
	add	w0, w0, 1	// tmp158, tmp159,
	str	w0, [sp, 72]	// tmp158, i
.L158:
	ldr	w0, [sp, 72]	// i.202_43, i
	ldr	w1, [sp, 76]	// tmp160, dlc
	cmp	w1, w0	// tmp160, i.202_43
	bhi	.L159		//,
	ldr	x0, [sp, 32]	// tmp161, flag
	ldr	w2, [x0]	// _44, *flag_52(D)
	ldr	x0, [sp, 56]	// tmp162, mod_reg_addr
	add	x0, x0, 64	// _45, tmp162,
	mov	x1, x0	//, _45
	mov	w0, w2	//, _44
	bl	writel		//
	nop	
	ldp	x29, x30, [sp], 80	//,,,
	hint	29 // autiasp
	ret	
	.size	awlink_asm_start_xmit, .-awlink_asm_start_xmit
	.align	2
	.global	awlink_asm_rx
	.type	awlink_asm_rx, %function
awlink_asm_rx:
	hint	25 // paciasp
	stp	x29, x30, [sp, -80]!	//,,,
	mov	x29, sp	//,
	str	x19, [sp, 16]	//,
	str	x0, [sp, 56]	// mod_reg_addr, mod_reg_addr
	str	x1, [sp, 48]	// r_fi, r_fi
	str	x2, [sp, 40]	// mod_reg_val, mod_reg_val
	str	x3, [sp, 32]	// id, id
	strb	wzr, [sp, 79]	//, fi
	ldr	x0, [sp, 56]	// tmp124, mod_reg_addr
	add	x0, x0, 64	// _1, tmp124,
	bl	readl		//
	and	w0, w0, 255	// _3, _2
	strb	w0, [sp, 79]	// tmp125, fi
	ldrb	w0, [sp, 79]	// tmp127, fi
	and	w0, w0, 255	// fi.203_4, tmp127
	sxtb	w0, w0	// fi.204_5, fi.203_4
	cmp	w0, 0	// fi.204_5,
	bge	.L161		//,
	ldr	x0, [sp, 40]	// tmp128, mod_reg_val
	mov	w1, 84	// tmp129,
	str	w1, [x0]	// tmp129, *mod_reg_val_39(D)
	bl	junk_fun17		//
	ldr	x0, [sp, 56]	// tmp130, mod_reg_addr
	add	x0, x0, 68	// _6, tmp130,
	bl	readl		//
	lsl	w19, w0, 21	// _8, _7,
	ldr	x0, [sp, 56]	// tmp131, mod_reg_addr
	add	x0, x0, 72	// _9, tmp131,
	bl	readl		//
	lsl	w0, w0, 13	// _11, _10,
	orr	w19, w19, w0	// _12, _8, _11
	ldr	x0, [sp, 56]	// tmp132, mod_reg_addr
	add	x0, x0, 76	// _13, tmp132,
	bl	readl		//
	lsl	w0, w0, 5	// _15, _14,
	orr	w19, w19, w0	// _16, _12, _15
	ldr	x0, [sp, 56]	// tmp133, mod_reg_addr
	add	x0, x0, 80	// _17, tmp133,
	bl	readl		//
	lsr	w0, w0, 3	// _19, _18,
	and	w0, w0, 31	// _20, _19,
	orr	w1, w19, w0	// _21, _16, _20
	ldr	x0, [sp, 32]	// tmp134, id
	str	w1, [x0]	// _21, *id_43(D)
	ldr	x0, [sp, 32]	// tmp135, id
	ldr	w0, [x0]	// _22, *id_43(D)
	orr	w1, w0, -2147483648	// _23, _22,
	ldr	x0, [sp, 32]	// tmp136, id
	str	w1, [x0]	// _23, *id_43(D)
	b	.L162		//
.L161:
	ldr	x0, [sp, 40]	// tmp137, mod_reg_val
	mov	w1, 76	// tmp138,
	str	w1, [x0]	// tmp138, *mod_reg_val_39(D)
	ldr	x0, [sp, 56]	// tmp139, mod_reg_addr
	add	x0, x0, 68	// _24, tmp139,
	bl	readl		//
	lsl	w19, w0, 3	// _26, _25,
	ldr	x0, [sp, 56]	// tmp140, mod_reg_addr
	add	x0, x0, 72	// _27, tmp140,
	bl	readl		//
	lsr	w0, w0, 5	// _29, _28,
	and	w0, w0, 7	// _30, _29,
	orr	w1, w19, w0	// _31, _26, _30
	ldr	x0, [sp, 32]	// tmp141, id
	str	w1, [x0]	// _31, *id_43(D)
.L162:
	ldrb	w0, [sp, 79]	// tmp143, fi
	and	w1, w0, 255	// fi.205_32, tmp143
	ldr	x0, [sp, 48]	// tmp144, r_fi
	strb	w1, [x0]	// tmp145, *r_fi_53(D)
	nop	
	ldr	x19, [sp, 16]	//,
	ldp	x29, x30, [sp], 80	//,,,
	hint	29 // autiasp
	ret	
	.size	awlink_asm_rx, .-awlink_asm_rx
	.section	.rodata
	.align	3
.LC8:
	.string	"reg"
	.text
	.align	2
	.global	awlink_asm_probe
	.type	awlink_asm_probe, %function
awlink_asm_probe:
	hint	25 // paciasp
	sub	sp, sp, #176	//,,
	stp	x29, x30, [sp, 16]	//,,
	add	x29, sp, 16	//,,
	str	x0, [sp, 40]	// node, node
	str	x1, [sp, 32]	// num, num
	mrs	x0, sp_el0	// tmp100
	add	x0, x0, 1048	// tmp101, tmp100,
	ldr	x1, [x0]	// tmp134,
	str	x1, [sp, 168]	// tmp134, D.69910
	mov	x1, 0	// tmp134
	mov	w0, -1	// tmp102,
	str	w0, [sp, 60]	// tmp102, data
	mov	w0, 35	// tmp103,
	str	w0, [sp, 64]	// tmp103, mod_reg_val
	mov	w0, 1	// tmp104,
	str	w0, [sp, 68]	// tmp104, mod_reg_addr
	adrp	x0, res	// tmp106,
	add	x0, x0, :lo12:res	// tmp105, tmp106,
	mov	x1, -1	// tmp107,
	str	x1, [x0]	// tmp107, res.a0
	str	xzr, [sp, 8]	//,
	adrp	x0, res	// tmp109,
	add	x0, x0, :lo12:res	// tmp108, tmp109,
	str	x0, [sp]	// tmp108,
	mov	x7, 0	//,
	mov	x6, 0	//,
	mov	x5, 0	//,
	mov	x4, 0	//,
	mov	x3, 0	//,
	ldr	x2, [sp, 32]	//, num
	mov	w1, 4294901762	//,
	mov	x0, 160	//,
	movk	x0, 0xc000, lsl 16	//,,
	bl	__arm_smccc_smc		//
	adrp	x0, res	// tmp111,
	add	x0, x0, :lo12:res	// tmp110, tmp111,
	ldr	x0, [x0]	// _1, res.a0
	cmp	x0, 0	// _1,
	bne	.L164		//,
	mov	x2, 0	//,
	adrp	x0, .LC8	// tmp112,
	add	x1, x0, :lo12:.LC8	//, tmp112,
	ldr	x0, [sp, 40]	//, node
	bl	of_get_property		//
	str	x0, [sp, 72]	//, be_addr
	ldr	x0, [sp, 40]	//, node
	bl	of_irq_find_parent		//
	str	x0, [sp, 80]	//, p
	ldr	x0, [sp, 80]	// tmp113, p
	cmp	x0, 0	// tmp113,
	bne	.L165		//,
	mov	w0, -1	// _8,
	b	.L167		//
.L165:
	ldr	x0, [sp, 80]	// tmp114, p
	str	x0, [sp, 88]	// tmp114, oirq.np
	mov	w0, 3	// tmp115,
	str	w0, [sp, 96]	// tmp115, oirq.args_count
	str	wzr, [sp, 100]	//, oirq.args[0]
	ldr	w1, [sp, 64]	// tmp117, mod_reg_val
	ldr	w0, [sp, 68]	// tmp118, mod_reg_addr
	add	w0, w1, w0	// tmp116, tmp117, tmp118
	str	w0, [sp, 64]	// tmp116, mod_reg_val
	bl	junk_fun3		//
	ldr	x1, [sp, 32]	// tmp119, num
	mov	x0, x1	// tmp120, tmp119
	lsl	x0, x0, 6	// tmp121, tmp120,
	add	x0, x0, x1	// tmp120, tmp120, tmp119
	lsl	x0, x0, 1	// tmp122, tmp120,
	str	x0, [sp, 32]	// tmp120, num
	ldr	x0, [sp, 32]	// tmp123, num
	mov	w1, w0	// _2, tmp123
	ldr	w0, [sp, 64]	// mod_reg_val.206_3, mod_reg_val
	add	w0, w1, w0	// _4, _2, mod_reg_val.206_3
	str	w0, [sp, 64]	// _4, mod_reg_val
	ldr	w2, [sp, 64]	// mod_reg_val.207_5, mod_reg_val
	ldrsw	x0, [sp, 68]	// tmp124, mod_reg_addr
	lsl	x0, x0, 2	// tmp125, tmp124,
	add	x1, sp, 100	// tmp126,,
	str	w2, [x1, x0]	// mod_reg_val.207_5, oirq.args[mod_reg_addr_13]
	bl	junk_fun15		//
	mov	w0, 4	// tmp127,
	str	w0, [sp, 108]	// tmp127, oirq.args[2]
	add	x0, sp, 88	// tmp128,,
	mov	x1, x0	//, tmp128
	ldr	x0, [sp, 72]	//, be_addr
	bl	of_irq_parse_raw		//
	bl	junk_fun16		//
	add	x0, sp, 88	// tmp129,,
	bl	irq_create_of_mapping		//
	str	w0, [sp, 60]	// _6, data
.L164:
	ldr	w0, [sp, 60]	// _8, data
.L167:
	mov	w1, w0	// <retval>, _8
	mrs	x0, sp_el0	// tmp131
	add	x0, x0, 1048	// tmp132, tmp131,
	ldr	x2, [sp, 168]	// tmp135, D.69910
	ldr	x3, [x0]	// tmp136,
	subs	x2, x2, x3	// tmp135, tmp136
	mov	x3, 0	// tmp136
	beq	.L168		//,
	bl	__stack_chk_fail		//
.L168:
	mov	w0, w1	//, <retval>
	ldp	x29, x30, [sp, 16]	//,,
	add	sp, sp, 176	//,,
	hint	29 // autiasp
	ret	
	.size	awlink_asm_probe, .-awlink_asm_probe
	.ident	"GCC: (GNU Toolchain for the A-profile Architecture 10.3-2021.07 (arm-10.29)) 10.3.1 20210621"
	.section	.note.GNU-stack,"",@progbits
	.section	.note.gnu.property,"a"
	.align	3
	.word	4
	.word	16
	.word	5
	.string	"GNU"
	.word	3221225472
	.word	4
	.word	2
	.align	3
