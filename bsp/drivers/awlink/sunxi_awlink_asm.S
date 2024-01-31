	.text
	.section	.note.gnu.property,"a",@note
	.p2align	3
	.word	4
	.word	16
	.word	5
	.asciz	"GNU"
	.word	3221225472
	.word	4
	.word	3
	.word	0
.Lsec_end0:
	.text
	.file	"sunxi_awlink_asm.c"
	.globl	awlink_asm_write_cmdreg         // -- Begin function awlink_asm_write_cmdreg
	.p2align	2
	.type	awlink_asm_write_cmdreg,@function
awlink_asm_write_cmdreg:                // @awlink_asm_write_cmdreg
// %bb.0:
	hint	#25
	sub	sp, sp, #48
	str	x30, [x18], #8
	stp	x29, x30, [sp, #32]             // 16-byte Folded Spill
	add	x29, sp, #32
	stur	w0, [x29, #-4]
	str	x1, [sp, #16]
	str	x2, [sp, #8]
	bl	junk_fun1
	ldur	w0, [x29, #-4]
	ldr	x8, [sp, #16]
	add	x1, x8, #4
	bl	writel
	bl	junk_fun2
	ldp	x29, x30, [sp, #32]             // 16-byte Folded Reload
	ldr	x30, [x18, #-8]!
	add	sp, sp, #48
	hint	#29
	ret
.Lfunc_end0:
	.size	awlink_asm_write_cmdreg, .Lfunc_end0-awlink_asm_write_cmdreg
                                        // -- End function
	.p2align	2                               // -- Begin function junk_fun1
	.type	junk_fun1,@function
junk_fun1:                              // @junk_fun1
// %bb.0:
	hint	#25
	sub	sp, sp, #16
	mov	w8, wzr
	str	w8, [sp, #8]
	mov	w9, #15
	str	w9, [sp, #8]
	str	w8, [sp, #4]
	mov	w8, #10
	str	w8, [sp, #4]
	ldr	w8, [sp, #8]
	ldr	w9, [sp, #4]
	subs	w8, w8, w9
	b.ge	.LBB1_2
	b	.LBB1_1
.LBB1_1:
	ldr	w8, [sp, #4]
	str	w8, [sp, #12]
	b	.LBB1_5
.LBB1_2:
	ldr	w8, [sp, #4]
	subs	w8, w8, #16
	b.ne	.LBB1_4
	b	.LBB1_3
.LBB1_3:
	ldr	w8, [sp, #8]
	str	w8, [sp, #12]
	b	.LBB1_5
.LBB1_4:
	ldr	w9, [sp, #4]
	ldr	w8, [sp, #8]
	eor	w8, w8, w9
	str	w8, [sp, #8]
	ldr	w8, [sp, #8]
	str	w8, [sp, #12]
	b	.LBB1_5
.LBB1_5:
	ldr	w0, [sp, #12]
	add	sp, sp, #16
	hint	#29
	ret
.Lfunc_end1:
	.size	junk_fun1, .Lfunc_end1-junk_fun1
                                        // -- End function
	.p2align	2                               // -- Begin function writel
	.type	writel,@function
writel:                                 // @writel
// %bb.0:
	hint	#25
	sub	sp, sp, #96
	str	x30, [x18], #8
	stp	x29, x30, [sp, #80]             // 16-byte Folded Spill
	add	x29, sp, #80
	stur	w0, [x29, #-20]
	stur	x1, [x29, #-32]
	ldur	w8, [x29, #-20]
	mov	w0, w8
	str	x0, [sp, #16]                   // 8-byte Folded Spill
	ldur	x0, [x29, #-32]
	str	x0, [sp, #24]                   // 8-byte Folded Spill
	b	.LBB2_1
.Ltmp0:                                 // Block address taken
.LBB2_1:
	hint	#36
	ldr	x2, [sp, #24]                   // 8-byte Folded Reload
	ldr	x0, [sp, #16]                   // 8-byte Folded Reload
	adrp	x8, .Ltmp0
	add	x8, x8, :lo12:.Ltmp0
	str	x8, [sp, #40]
	ldr	x3, [sp, #40]
	mov	w1, #32
	bl	log_write_mmio
	//APP
	dmb	oshst
	//NO_APP
	ldur	w9, [x29, #-20]
	ldur	x8, [x29, #-32]
	stur	w9, [x29, #-4]
	stur	x8, [x29, #-16]
	ldur	w8, [x29, #-4]
	ldur	x9, [x29, #-16]
	//APP
	str	w8, [x9]
	//NO_APP
	ldur	w8, [x29, #-20]
	mov	w0, w8
	str	x0, [sp]                        // 8-byte Folded Spill
	ldur	x0, [x29, #-32]
	str	x0, [sp, #8]                    // 8-byte Folded Spill
	b	.LBB2_2
.Ltmp1:                                 // Block address taken
.LBB2_2:
	hint	#36
	ldr	x2, [sp, #8]                    // 8-byte Folded Reload
	ldr	x0, [sp]                        // 8-byte Folded Reload
	adrp	x8, .Ltmp1
	add	x8, x8, :lo12:.Ltmp1
	str	x8, [sp, #32]
	ldr	x3, [sp, #32]
	mov	w1, #32
	bl	log_post_write_mmio
	ldp	x29, x30, [sp, #80]             // 16-byte Folded Reload
	ldr	x30, [x18, #-8]!
	add	sp, sp, #96
	hint	#29
	ret
.Lfunc_end2:
	.size	writel, .Lfunc_end2-writel
                                        // -- End function
	.p2align	2                               // -- Begin function junk_fun2
	.type	junk_fun2,@function
junk_fun2:                              // @junk_fun2
// %bb.0:
	hint	#25
	sub	sp, sp, #16
	mov	w9, wzr
	str	w9, [sp, #8]
	mov	w8, #2
	str	w8, [sp, #8]
	str	w9, [sp, #4]
	str	w8, [sp, #4]
	ldr	w8, [sp, #4]
	and	w8, w8, #0x1
	subs	w8, w8, #768
	b.eq	.LBB3_2
	b	.LBB3_1
.LBB3_1:
	ldr	w9, [sp, #8]
	ldr	w8, [sp, #4]
	add	w8, w8, w9
	str	w8, [sp, #4]
	b	.LBB3_2
.LBB3_2:
	ldr	w8, [sp, #4]
	mov	w9, #819
	mul	w8, w8, w9
	str	w8, [sp, #8]
	ldr	w8, [sp, #8]
	and	w8, w8, #0x2
	str	w8, [sp, #4]
	ldr	w8, [sp, #8]
	subs	w8, w8, #819
	b.ne	.LBB3_4
	b	.LBB3_3
.LBB3_3:
	ldr	w8, [sp, #4]
	str	w8, [sp, #12]
	b	.LBB3_5
.LBB3_4:
	ldr	w8, [sp, #8]
	str	w8, [sp, #12]
	b	.LBB3_5
.LBB3_5:
	ldr	w0, [sp, #12]
	add	sp, sp, #16
	hint	#29
	ret
.Lfunc_end3:
	.size	junk_fun2, .Lfunc_end3-junk_fun2
                                        // -- End function
	.globl	awlink_asm_fun0                 // -- Begin function awlink_asm_fun0
	.p2align	2
	.type	awlink_asm_fun0,@function
awlink_asm_fun0:                        // @awlink_asm_fun0
// %bb.0:
	hint	#25
	sub	sp, sp, #48
	str	x30, [x18], #8
	stp	x29, x30, [sp, #32]             // 16-byte Folded Spill
	add	x29, sp, #32
	stur	x0, [x29, #-8]
	str	x1, [sp, #16]
	ldur	x2, [x29, #-8]
	ldr	x4, [sp, #16]
	mov	x9, sp
	mov	x7, xzr
	str	x7, [x9, #8]
	adrp	x8, res
	add	x8, x8, :lo12:res
	str	x8, [x9]
	mov	w8, #160
	movk	w8, #49152, lsl #16
	mov	w0, w8
	mov	w8, #-65529
	mov	w1, w8
	mov	x3, x7
	mov	x5, x7
	mov	x6, x7
	bl	__arm_smccc_smc
	ldp	x29, x30, [sp, #32]             // 16-byte Folded Reload
	ldr	x30, [x18, #-8]!
	add	sp, sp, #48
	hint	#29
	ret
.Lfunc_end4:
	.size	awlink_asm_fun0, .Lfunc_end4-awlink_asm_fun0
                                        // -- End function
	.globl	awlink_asm_fun1                 // -- Begin function awlink_asm_fun1
	.p2align	2
	.type	awlink_asm_fun1,@function
awlink_asm_fun1:                        // @awlink_asm_fun1
// %bb.0:
	hint	#25
	sub	sp, sp, #48
	str	x30, [x18], #8
	stp	x29, x30, [sp, #32]             // 16-byte Folded Spill
	add	x29, sp, #32
	stur	x0, [x29, #-8]
	ldur	x2, [x29, #-8]
	mov	x9, sp
	mov	x7, xzr
	str	x7, [x9, #8]
	adrp	x8, res
	add	x8, x8, :lo12:res
	str	x8, [x9]
	mov	w8, #160
	movk	w8, #49152, lsl #16
	mov	w0, w8
	mov	w8, #-65533
	mov	w1, w8
	mov	x3, x7
	mov	x4, x7
	mov	x5, x7
	mov	x6, x7
	bl	__arm_smccc_smc
	ldp	x29, x30, [sp, #32]             // 16-byte Folded Reload
	ldr	x30, [x18, #-8]!
	add	sp, sp, #48
	hint	#29
	ret
.Lfunc_end5:
	.size	awlink_asm_fun1, .Lfunc_end5-awlink_asm_fun1
                                        // -- End function
	.globl	awlink_asm_fun2                 // -- Begin function awlink_asm_fun2
	.p2align	2
	.type	awlink_asm_fun2,@function
awlink_asm_fun2:                        // @awlink_asm_fun2
// %bb.0:
	hint	#25
	sub	sp, sp, #48
	str	x30, [x18], #8
	stp	x29, x30, [sp, #32]             // 16-byte Folded Spill
	add	x29, sp, #32
	stur	x0, [x29, #-8]
	ldur	x2, [x29, #-8]
	mov	x9, sp
	mov	x7, xzr
	str	x7, [x9, #8]
	adrp	x8, res
	add	x8, x8, :lo12:res
	str	x8, [x9]
	mov	w8, #160
	movk	w8, #49152, lsl #16
	mov	w0, w8
	mov	w8, #-65532
	mov	w1, w8
	mov	x3, x7
	mov	x4, x7
	mov	x5, x7
	mov	x6, x7
	bl	__arm_smccc_smc
	ldp	x29, x30, [sp, #32]             // 16-byte Folded Reload
	ldr	x30, [x18, #-8]!
	add	sp, sp, #48
	hint	#29
	ret
.Lfunc_end6:
	.size	awlink_asm_fun2, .Lfunc_end6-awlink_asm_fun2
                                        // -- End function
	.globl	awlink_asm_fun3                 // -- Begin function awlink_asm_fun3
	.p2align	2
	.type	awlink_asm_fun3,@function
awlink_asm_fun3:                        // @awlink_asm_fun3
// %bb.0:
	hint	#25
	sub	sp, sp, #48
	str	x30, [x18], #8
	stp	x29, x30, [sp, #32]             // 16-byte Folded Spill
	add	x29, sp, #32
	stur	x0, [x29, #-8]
	mov	x9, sp
	mov	x7, xzr
	str	x7, [x9, #8]
	adrp	x8, res
	add	x8, x8, :lo12:res
	str	x8, [x9]
	mov	w8, #160
	movk	w8, #49152, lsl #16
	mov	w0, w8
	mov	w8, #-65531
	mov	w1, w8
	mov	x2, x7
	mov	x3, x7
	mov	x4, x7
	mov	x5, x7
	mov	x6, x7
	bl	__arm_smccc_smc
	ldp	x29, x30, [sp, #32]             // 16-byte Folded Reload
	ldr	x30, [x18, #-8]!
	add	sp, sp, #48
	hint	#29
	ret
.Lfunc_end7:
	.size	awlink_asm_fun3, .Lfunc_end7-awlink_asm_fun3
                                        // -- End function
	.globl	awlink_asm_fun4                 // -- Begin function awlink_asm_fun4
	.p2align	2
	.type	awlink_asm_fun4,@function
awlink_asm_fun4:                        // @awlink_asm_fun4
// %bb.0:
	hint	#25
	sub	sp, sp, #48
	str	x30, [x18], #8
	stp	x29, x30, [sp, #32]             // 16-byte Folded Spill
	add	x29, sp, #32
	stur	x0, [x29, #-8]
	mov	x9, sp
	mov	x7, xzr
	str	x7, [x9, #8]
	adrp	x8, res
	add	x8, x8, :lo12:res
	str	x8, [x9]
	mov	w8, #160
	movk	w8, #49152, lsl #16
	mov	w0, w8
	mov	w8, #-65530
	mov	w1, w8
	mov	x2, x7
	mov	x3, x7
	mov	x4, x7
	mov	x5, x7
	mov	x6, x7
	bl	__arm_smccc_smc
	ldp	x29, x30, [sp, #32]             // 16-byte Folded Reload
	ldr	x30, [x18, #-8]!
	add	sp, sp, #48
	hint	#29
	ret
.Lfunc_end8:
	.size	awlink_asm_fun4, .Lfunc_end8-awlink_asm_fun4
                                        // -- End function
	.globl	awlink_asm_fun5                 // -- Begin function awlink_asm_fun5
	.p2align	2
	.type	awlink_asm_fun5,@function
awlink_asm_fun5:                        // @awlink_asm_fun5
// %bb.0:
	hint	#25
	sub	sp, sp, #64
	str	x30, [x18], #8
	stp	x29, x30, [sp, #48]             // 16-byte Folded Spill
	add	x29, sp, #48
	stur	x0, [x29, #-8]
	ldur	x2, [x29, #-8]
	mov	x9, sp
	mov	x7, xzr
	str	x7, [x9, #8]
	adrp	x8, res
	add	x8, x8, :lo12:res
	str	x8, [sp, #24]                   // 8-byte Folded Spill
	str	x8, [x9]
	mov	w8, #160
	movk	w8, #49152, lsl #16
	mov	w0, w8
	mov	w8, #-65528
	mov	w1, w8
	mov	x3, x7
	mov	x4, x7
	mov	x5, x7
	mov	x6, x7
	bl	__arm_smccc_smc
	ldr	x8, [sp, #24]                   // 8-byte Folded Reload
	ldr	x0, [x8]
	ldr	x1, [x8, #8]
	adrp	x8, arm64_use_ng_mappings
	ldrb	w8, [x8, :lo12:arm64_use_ng_mappings]
	ands	w8, w8, #0x1
	mov	x9, #1811
	movk	x9, #104, lsl #48
	mov	x8, #3859
	movk	x8, #104, lsl #48
	csel	x8, x8, x9, ne
	stur	x8, [x29, #-16]
	ldur	x2, [x29, #-16]
	bl	__ioremap
	ldp	x29, x30, [sp, #48]             // 16-byte Folded Reload
	ldr	x30, [x18, #-8]!
	add	sp, sp, #64
	hint	#29
	ret
.Lfunc_end9:
	.size	awlink_asm_fun5, .Lfunc_end9-awlink_asm_fun5
                                        // -- End function
	.globl	awlink_asm_start                // -- Begin function awlink_asm_start
	.p2align	2
	.type	awlink_asm_start,@function
awlink_asm_start:                       // @awlink_asm_start
// %bb.0:
	hint	#25
	sub	sp, sp, #64
	str	x30, [x18], #8
	stp	x29, x30, [sp, #48]             // 16-byte Folded Spill
	add	x29, sp, #48
	stur	x0, [x29, #-8]
	stur	x1, [x29, #-16]
	str	x2, [sp, #24]
	ldr	x2, [sp, #24]
	ldur	x3, [x29, #-16]
	mov	x9, sp
	mov	x7, xzr
	str	x7, [x9, #8]
	adrp	x8, res
	add	x8, x8, :lo12:res
	str	x8, [x9]
	mov	w8, #160
	movk	w8, #49152, lsl #16
	mov	w0, w8
	mov	w8, #-65535
	mov	w1, w8
	mov	x4, x7
	mov	x5, x7
	mov	x6, x7
	bl	__arm_smccc_smc
	bl	junk_fun4
	ldp	x29, x30, [sp, #48]             // 16-byte Folded Reload
	ldr	x30, [x18, #-8]!
	add	sp, sp, #64
	hint	#29
	ret
.Lfunc_end10:
	.size	awlink_asm_start, .Lfunc_end10-awlink_asm_start
                                        // -- End function
	.p2align	2                               // -- Begin function junk_fun4
	.type	junk_fun4,@function
junk_fun4:                              // @junk_fun4
// %bb.0:
	hint	#25
	sub	sp, sp, #16
	mov	w8, wzr
	str	w8, [sp, #12]
	mov	w9, #10
	str	w9, [sp, #12]
	str	w8, [sp, #8]
	mov	w8, #7
	str	w8, [sp, #8]
	ldr	w8, [sp, #8]
	mov	w9, #819
	mul	w8, w8, w9
	str	w8, [sp, #12]
	ldr	w9, [sp, #8]
	ldr	w8, [sp, #12]
	add	w8, w8, w9
	str	w8, [sp, #12]
	ldr	w0, [sp, #12]
	add	sp, sp, #16
	hint	#29
	ret
.Lfunc_end11:
	.size	junk_fun4, .Lfunc_end11-junk_fun4
                                        // -- End function
	.globl	awlink_asm_set_bittiming        // -- Begin function awlink_asm_set_bittiming
	.p2align	2
	.type	awlink_asm_set_bittiming,@function
awlink_asm_set_bittiming:               // @awlink_asm_set_bittiming
// %bb.0:
	hint	#25
	sub	sp, sp, #48
	str	x30, [x18], #8
	stp	x29, x30, [sp, #32]             // 16-byte Folded Spill
	add	x29, sp, #32
	stur	x0, [x29, #-8]
	stur	w1, [x29, #-12]
	str	x2, [sp, #8]
	str	x3, [sp]
	bl	junk_fun9
	ldurb	w8, [x29, #-12]
	tbz	w8, #2, .LBB12_2
	b	.LBB12_1
.LBB12_1:
	ldr	x9, [sp, #8]
	ldr	w8, [x9]
	orr	w8, w8, #0x800000
	str	w8, [x9]
	b	.LBB12_2
.LBB12_2:
	bl	junk_fun10
	ldr	x8, [sp, #8]
	ldr	w0, [x8]
	ldur	x8, [x29, #-8]
	add	x1, x8, #20
	bl	writel
	ldp	x29, x30, [sp, #32]             // 16-byte Folded Reload
	ldr	x30, [x18, #-8]!
	add	sp, sp, #48
	hint	#29
	ret
.Lfunc_end12:
	.size	awlink_asm_set_bittiming, .Lfunc_end12-awlink_asm_set_bittiming
                                        // -- End function
	.p2align	2                               // -- Begin function junk_fun9
	.type	junk_fun9,@function
junk_fun9:                              // @junk_fun9
// %bb.0:
	hint	#25
	sub	sp, sp, #16
	mov	w8, wzr
	str	w8, [sp, #8]
	mov	w9, #15
	str	w9, [sp, #8]
	str	w8, [sp, #4]
	mov	w8, #12
	str	w8, [sp, #4]
	ldr	w8, [sp, #8]
	add	w8, w8, #1
	str	w8, [sp, #8]
	ldr	w8, [sp, #4]
	add	w8, w8, #51
	str	w8, [sp, #4]
	ldr	w8, [sp, #4]
	subs	w8, w8, #7
	b.ne	.LBB13_2
	b	.LBB13_1
.LBB13_1:
	ldr	w8, [sp, #4]
	str	w8, [sp, #12]
	b	.LBB13_7
.LBB13_2:
	ldr	w8, [sp, #8]
	add	w9, w8, #1
	str	w9, [sp, #8]
	tbnz	w8, #3, .LBB13_4
	b	.LBB13_3
.LBB13_3:
	ldr	w9, [sp, #8]
	ldr	w8, [sp, #4]
	mul	w8, w8, w9
	str	w8, [sp, #4]
	b	.LBB13_4
.LBB13_4:
	ldr	w8, [sp, #4]
	subs	w8, w8, #16
	b.ne	.LBB13_6
	b	.LBB13_5
.LBB13_5:
	ldr	w8, [sp, #8]
	str	w8, [sp, #12]
	b	.LBB13_7
.LBB13_6:
	ldr	w8, [sp, #8]
	mov	w9, #243
	and	w8, w8, w9
	str	w8, [sp, #4]
	ldr	w8, [sp, #8]
	str	w8, [sp, #12]
	b	.LBB13_7
.LBB13_7:
	ldr	w0, [sp, #12]
	add	sp, sp, #16
	hint	#29
	ret
.Lfunc_end13:
	.size	junk_fun9, .Lfunc_end13-junk_fun9
                                        // -- End function
	.p2align	2                               // -- Begin function junk_fun10
	.type	junk_fun10,@function
junk_fun10:                             // @junk_fun10
// %bb.0:
	hint	#25
	sub	sp, sp, #16
	mov	w8, wzr
	str	w8, [sp, #8]
	mov	w9, #9
	str	w9, [sp, #8]
	str	w8, [sp, #4]
	mov	w8, #11
	str	w8, [sp, #4]
	ldr	w8, [sp, #8]
	add	w8, w8, #1
	str	w8, [sp, #8]
	ldr	w8, [sp, #4]
	add	w8, w8, #51
	str	w8, [sp, #4]
	ldr	w8, [sp, #4]
	subs	w8, w8, #7
	b.ne	.LBB14_2
	b	.LBB14_1
.LBB14_1:
	ldr	w8, [sp, #4]
	str	w8, [sp, #12]
	b	.LBB14_3
.LBB14_2:
	ldr	w8, [sp, #8]
	mov	w9, #243
	and	w8, w8, w9
	str	w8, [sp, #4]
	ldr	w8, [sp, #8]
	str	w8, [sp, #12]
	b	.LBB14_3
.LBB14_3:
	ldr	w0, [sp, #12]
	add	sp, sp, #16
	hint	#29
	ret
.Lfunc_end14:
	.size	junk_fun10, .Lfunc_end14-junk_fun10
                                        // -- End function
	.globl	awlink_asm_clean_transfer_err   // -- Begin function awlink_asm_clean_transfer_err
	.p2align	2
	.type	awlink_asm_clean_transfer_err,@function
awlink_asm_clean_transfer_err:          // @awlink_asm_clean_transfer_err
// %bb.0:
	hint	#25
	sub	sp, sp, #48
	str	x30, [x18], #8
	stp	x29, x30, [sp, #32]             // 16-byte Folded Spill
	add	x29, sp, #32
	stur	x0, [x29, #-8]
	str	x1, [sp, #16]
	str	x2, [sp, #8]
	mov	w8, wzr
	str	w8, [sp, #4]
	bl	junk_fun12
	ldur	x8, [x29, #-8]
	add	x0, x8, #28
	bl	readl
	str	w0, [sp, #4]
	bl	junk_fun13
	ldrb	w8, [sp, #4]
	ldr	x9, [sp, #16]
	strh	w8, [x9]
	ldrb	w8, [sp, #6]
	ldr	x9, [sp, #8]
	strh	w8, [x9]
	ldp	x29, x30, [sp, #32]             // 16-byte Folded Reload
	ldr	x30, [x18, #-8]!
	add	sp, sp, #48
	hint	#29
	ret
.Lfunc_end15:
	.size	awlink_asm_clean_transfer_err, .Lfunc_end15-awlink_asm_clean_transfer_err
                                        // -- End function
	.p2align	2                               // -- Begin function junk_fun12
	.type	junk_fun12,@function
junk_fun12:                             // @junk_fun12
// %bb.0:
	hint	#25
	sub	sp, sp, #16
	mov	w8, wzr
	str	w8, [sp, #8]
	mov	w9, #6
	str	w9, [sp, #8]
	str	w8, [sp, #4]
	mov	w8, #11
	str	w8, [sp, #4]
	ldr	w9, [sp, #4]
	ldr	w8, [sp, #8]
	eor	w8, w8, w9
	str	w8, [sp, #8]
	ldr	w8, [sp, #4]
	mov	w9, #819
	mul	w8, w8, w9
	str	w8, [sp, #8]
	ldr	w9, [sp, #4]
	ldr	w8, [sp, #8]
	eor	w8, w8, w9
	str	w8, [sp, #8]
	ldr	w8, [sp, #8]
	subs	w8, w8, #7
	b.ne	.LBB16_2
	b	.LBB16_1
.LBB16_1:
	ldr	w8, [sp, #4]
	str	w8, [sp, #12]
	b	.LBB16_3
.LBB16_2:
	ldr	w8, [sp, #8]
	str	w8, [sp, #12]
	b	.LBB16_3
.LBB16_3:
	ldr	w0, [sp, #12]
	add	sp, sp, #16
	hint	#29
	ret
.Lfunc_end16:
	.size	junk_fun12, .Lfunc_end16-junk_fun12
                                        // -- End function
	.p2align	2                               // -- Begin function readl
	.type	readl,@function
readl:                                  // @readl
// %bb.0:
	hint	#25
	sub	sp, sp, #96
	str	x30, [x18], #8
	stp	x29, x30, [sp, #80]             // 16-byte Folded Spill
	add	x29, sp, #80
	stur	x0, [x29, #-24]
	mov	w8, wzr
	stur	w8, [x29, #-28]
	ldur	x0, [x29, #-24]
	str	x0, [sp, #16]                   // 8-byte Folded Spill
	b	.LBB17_1
.Ltmp2:                                 // Block address taken
.LBB17_1:
	hint	#36
	ldr	x1, [sp, #16]                   // 8-byte Folded Reload
	adrp	x8, .Ltmp2
	add	x8, x8, :lo12:.Ltmp2
	str	x8, [sp, #40]
	ldr	x2, [sp, #40]
	mov	w0, #32
	bl	log_read_mmio
	ldur	x8, [x29, #-24]
	stur	x8, [x29, #-8]
	mov	w8, wzr
	stur	w8, [x29, #-12]
	ldur	x8, [x29, #-8]
	//APP
.Ltmp3:
	ldr	w8, [x8]
.Ltmp4:
	.section	.altinstructions,"a",@progbits
.Ltmp5:
	.word	.Ltmp3-.Ltmp5
.Ltmp7:
	.word	.Ltmp6-.Ltmp7
	.hword	61
	.byte	.Ltmp4-.Ltmp3
	.byte	.Ltmp8-.Ltmp6
	.text

	.text	1
.Ltmp6:
	ldar	w8, [x8]
.Ltmp8:
.Ltmp9:
.org (.Ltmp9-(.Ltmp8-.Ltmp6))+(.Ltmp4-.Ltmp3), 0
.Ltmp10:
.org (.Ltmp10-(.Ltmp4-.Ltmp3))+(.Ltmp8-.Ltmp6), 0
	.text


	//NO_APP
	stur	w8, [x29, #-12]
	ldur	w8, [x29, #-12]
	stur	w8, [x29, #-28]
	mov	x8, xzr
	str	x8, [sp, #32]
	//APP
	dmb	oshld
	//NO_APP
	ldur	w8, [x29, #-28]
                                        // kill: def $x8 killed $w8
	//APP
	eor	x8, x8, x8
.Ltmp11:
	cbnz	x8, .Ltmp11
	//NO_APP
	str	x8, [sp, #32]
	ldur	w8, [x29, #-28]
	mov	w0, w8
	str	x0, [sp]                        // 8-byte Folded Spill
	ldur	x0, [x29, #-24]
	str	x0, [sp, #8]                    // 8-byte Folded Spill
	b	.LBB17_2
.Ltmp12:                                // Block address taken
.LBB17_2:
	hint	#36
	ldr	x2, [sp, #8]                    // 8-byte Folded Reload
	ldr	x0, [sp]                        // 8-byte Folded Reload
	adrp	x8, .Ltmp12
	add	x8, x8, :lo12:.Ltmp12
	str	x8, [sp, #24]
	ldr	x3, [sp, #24]
	mov	w1, #32
	bl	log_post_read_mmio
	ldur	w0, [x29, #-28]
	ldp	x29, x30, [sp, #80]             // 16-byte Folded Reload
	ldr	x30, [x18, #-8]!
	add	sp, sp, #96
	hint	#29
	ret
.Lfunc_end17:
	.size	readl, .Lfunc_end17-readl
                                        // -- End function
	.p2align	2                               // -- Begin function junk_fun13
	.type	junk_fun13,@function
junk_fun13:                             // @junk_fun13
// %bb.0:
	hint	#25
	sub	sp, sp, #16
	mov	w8, wzr
	str	w8, [sp, #8]
	mov	w9, #8
	str	w9, [sp, #8]
	str	w8, [sp, #4]
	mov	w8, #15
	str	w8, [sp, #4]
	ldr	w8, [sp, #8]
	subs	w8, w8, #819
	b.ne	.LBB18_2
	b	.LBB18_1
.LBB18_1:
	ldr	w8, [sp, #4]
	str	w8, [sp, #12]
	b	.LBB18_7
.LBB18_2:
	ldr	w8, [sp, #8]
	mov	w9, #243
	and	w8, w8, w9
	str	w8, [sp, #4]
	ldr	w9, [sp, #4]
	ldr	w8, [sp, #8]
	add	w8, w8, w9
	str	w8, [sp, #8]
	ldr	w8, [sp, #8]
	add	w9, w8, #1
	str	w9, [sp, #8]
	tbnz	w8, #3, .LBB18_4
	b	.LBB18_3
.LBB18_3:
	ldr	w9, [sp, #8]
	ldr	w8, [sp, #4]
	mul	w8, w8, w9
	str	w8, [sp, #4]
	b	.LBB18_4
.LBB18_4:
	ldr	w8, [sp, #4]
	and	w8, w8, #0x1
	subs	w8, w8, #768
	b.eq	.LBB18_6
	b	.LBB18_5
.LBB18_5:
	ldr	w9, [sp, #8]
	ldr	w8, [sp, #4]
	add	w8, w8, w9
	str	w8, [sp, #4]
	b	.LBB18_6
.LBB18_6:
	ldr	w8, [sp, #8]
	str	w8, [sp, #12]
	b	.LBB18_7
.LBB18_7:
	ldr	w0, [sp, #12]
	add	sp, sp, #16
	hint	#29
	ret
.Lfunc_end18:
	.size	junk_fun13, .Lfunc_end18-junk_fun13
                                        // -- End function
	.globl	awlink_asm_start_xmit           // -- Begin function awlink_asm_start_xmit
	.p2align	2
	.type	awlink_asm_start_xmit,@function
awlink_asm_start_xmit:                  // @awlink_asm_start_xmit
// %bb.0:
	hint	#25
	sub	sp, sp, #80
	str	x30, [x18], #8
	stp	x29, x30, [sp, #64]             // 16-byte Folded Spill
	add	x29, sp, #64
	stur	x0, [x29, #-8]
	stur	x1, [x29, #-16]
	stur	w2, [x29, #-20]
	str	x3, [sp, #32]
	str	x4, [sp, #24]
	str	x5, [sp, #16]
	mov	w8, wzr
	str	w8, [sp, #12]
	str	w8, [sp, #12]
	str	w8, [sp, #8]
	ldr	x8, [sp, #32]
	ldr	w8, [x8]
	str	w8, [sp, #8]
	ldur	w8, [x29, #-20]
	tbz	w8, #30, .LBB19_2
	b	.LBB19_1
.LBB19_1:
	ldr	x9, [sp, #32]
	ldr	w8, [x9]
	orr	w8, w8, #0x40
	str	w8, [x9]
	b	.LBB19_2
.LBB19_2:
	ldur	w8, [x29, #-20]
	tbz	w8, #31, .LBB19_4
	b	.LBB19_3
.LBB19_3:
	ldr	x9, [sp, #32]
	ldr	w8, [x9]
	orr	w8, w8, #0x80
	str	w8, [x9]
	ldr	x9, [sp, #24]
	mov	w8, #84
	str	w8, [x9]
	ldur	w8, [x29, #-20]
	ubfx	w0, w8, #21, #8
	ldur	x8, [x29, #-8]
	add	x1, x8, #68
	bl	writel
	bl	junk_fun14
	ldur	w8, [x29, #-20]
	ubfx	w0, w8, #13, #8
	ldur	x8, [x29, #-8]
	add	x1, x8, #72
	bl	writel
	ldur	w8, [x29, #-20]
	ubfx	w0, w8, #5, #8
	ldur	x8, [x29, #-8]
	add	x1, x8, #76
	bl	writel
	ldur	w8, [x29, #-20]
	ubfiz	w0, w8, #3, #5
	ldur	x8, [x29, #-8]
	add	x1, x8, #80
	bl	writel
	b	.LBB19_5
.LBB19_4:
	ldr	x9, [sp, #24]
	mov	w8, #76
	str	w8, [x9]
	ldur	w8, [x29, #-20]
	ubfx	w0, w8, #3, #8
	ldur	x8, [x29, #-8]
	add	x1, x8, #68
	bl	writel
	ldur	w8, [x29, #-20]
	ubfiz	w0, w8, #5, #3
	ldur	x8, [x29, #-8]
	add	x1, x8, #72
	bl	writel
	b	.LBB19_5
.LBB19_5:
	mov	w8, wzr
	str	w8, [sp, #12]
	b	.LBB19_6
.LBB19_6:                               // =>This Inner Loop Header: Depth=1
	ldr	w8, [sp, #12]
	ldr	w9, [sp, #8]
	subs	w8, w8, w9
	b.hs	.LBB19_9
	b	.LBB19_7
.LBB19_7:                               //   in Loop: Header=BB19_6 Depth=1
	ldr	x8, [sp, #16]
	ldrsw	x9, [sp, #12]
	ldrb	w0, [x8, x9]
	ldur	x8, [x29, #-8]
	ldr	x9, [sp, #24]
	ldr	w9, [x9]
	ldr	w10, [sp, #12]
	add	w9, w9, w10, lsl #2
                                        // kill: def $x9 killed $w9
	add	x1, x8, x9
	bl	writel
	b	.LBB19_8
.LBB19_8:                               //   in Loop: Header=BB19_6 Depth=1
	ldr	w8, [sp, #12]
	add	w8, w8, #1
	str	w8, [sp, #12]
	b	.LBB19_6
.LBB19_9:
	ldr	x8, [sp, #32]
	ldr	w0, [x8]
	ldur	x8, [x29, #-8]
	add	x1, x8, #64
	bl	writel
	ldp	x29, x30, [sp, #64]             // 16-byte Folded Reload
	ldr	x30, [x18, #-8]!
	add	sp, sp, #80
	hint	#29
	ret
.Lfunc_end19:
	.size	awlink_asm_start_xmit, .Lfunc_end19-awlink_asm_start_xmit
                                        // -- End function
	.p2align	2                               // -- Begin function junk_fun14
	.type	junk_fun14,@function
junk_fun14:                             // @junk_fun14
// %bb.0:
	hint	#25
	sub	sp, sp, #16
	mov	w8, wzr
	str	w8, [sp, #8]
	mov	w9, #4
	str	w9, [sp, #8]
	str	w8, [sp, #4]
	mov	w8, #7
	str	w8, [sp, #4]
	ldr	w8, [sp, #8]
	ldr	w9, [sp, #4]
	subs	w8, w8, w9
	b.ge	.LBB20_2
	b	.LBB20_1
.LBB20_1:
	ldr	w8, [sp, #8]
	add	w8, w8, w8, lsl #1
	ldr	w9, [sp, #4]
	add	w8, w8, w9
	str	w8, [sp, #4]
	b	.LBB20_2
.LBB20_2:
	ldr	w8, [sp, #4]
	and	w8, w8, #0x1
	subs	w8, w8, #768
	b.eq	.LBB20_4
	b	.LBB20_3
.LBB20_3:
	ldr	w9, [sp, #8]
	ldr	w8, [sp, #4]
	add	w8, w8, w9
	str	w8, [sp, #4]
	b	.LBB20_4
.LBB20_4:
	ldr	w8, [sp, #8]
	ldr	w9, [sp, #4]
	subs	w8, w8, w9
	b.ge	.LBB20_6
	b	.LBB20_5
.LBB20_5:
	ldr	w8, [sp, #4]
	str	w8, [sp, #12]
	b	.LBB20_7
.LBB20_6:
	ldr	w8, [sp, #8]
	str	w8, [sp, #12]
	b	.LBB20_7
.LBB20_7:
	ldr	w0, [sp, #12]
	add	sp, sp, #16
	hint	#29
	ret
.Lfunc_end20:
	.size	junk_fun14, .Lfunc_end20-junk_fun14
                                        // -- End function
	.globl	awlink_asm_rx                   // -- Begin function awlink_asm_rx
	.p2align	2
	.type	awlink_asm_rx,@function
awlink_asm_rx:                          // @awlink_asm_rx
// %bb.0:
	hint	#25
	sub	sp, sp, #80
	str	x30, [x18], #8
	stp	x29, x30, [sp, #64]             // 16-byte Folded Spill
	add	x29, sp, #64
	stur	x0, [x29, #-8]
	stur	x1, [x29, #-16]
	stur	x2, [x29, #-24]
	str	x3, [sp, #32]
	mov	w8, wzr
	strb	w8, [sp, #28]
	strb	w8, [sp, #28]
	ldur	x8, [x29, #-8]
	add	x0, x8, #64
	bl	readl
	strb	w0, [sp, #28]
	ldrb	w8, [sp, #28]
	mov	w0, w8
	mov	w8, w0
	tbz	w8, #7, .LBB21_2
	b	.LBB21_1
.LBB21_1:
	ldur	x9, [x29, #-24]
	mov	w8, #84
	str	w8, [x9]
	bl	junk_fun17
	ldur	x8, [x29, #-8]
	add	x0, x8, #68
	bl	readl
	lsl	w8, w0, #21
	str	w8, [sp, #16]                   // 4-byte Folded Spill
	ldur	x8, [x29, #-8]
	add	x0, x8, #72
	bl	readl
	ldr	w8, [sp, #16]                   // 4-byte Folded Reload
	orr	w8, w8, w0, lsl #13
	str	w8, [sp, #20]                   // 4-byte Folded Spill
	ldur	x8, [x29, #-8]
	add	x0, x8, #76
	bl	readl
	ldr	w8, [sp, #20]                   // 4-byte Folded Reload
	orr	w8, w8, w0, lsl #5
	str	w8, [sp, #24]                   // 4-byte Folded Spill
	ldur	x8, [x29, #-8]
	add	x0, x8, #80
	bl	readl
	ldr	w8, [sp, #24]                   // 4-byte Folded Reload
	bfxil	w8, w0, #3, #5
	ldr	x9, [sp, #32]
	str	w8, [x9]
	ldr	x9, [sp, #32]
	ldr	w8, [x9]
	orr	w8, w8, #0x80000000
	str	w8, [x9]
	b	.LBB21_3
.LBB21_2:
	ldur	x9, [x29, #-24]
	mov	w8, #76
	str	w8, [x9]
	ldur	x8, [x29, #-8]
	add	x0, x8, #68
	bl	readl
	str	w0, [sp, #12]                   // 4-byte Folded Spill
	ldur	x8, [x29, #-8]
	add	x0, x8, #72
	bl	readl
	mov	w8, w0
	ldr	w0, [sp, #12]                   // 4-byte Folded Reload
	lsr	w8, w8, #5
	bfi	w8, w0, #3, #29
	ldr	x9, [sp, #32]
	str	w8, [x9]
	b	.LBB21_3
.LBB21_3:
	ldrb	w8, [sp, #28]
	ldur	x9, [x29, #-16]
	strb	w8, [x9]
	ldp	x29, x30, [sp, #64]             // 16-byte Folded Reload
	ldr	x30, [x18, #-8]!
	add	sp, sp, #80
	hint	#29
	ret
.Lfunc_end21:
	.size	awlink_asm_rx, .Lfunc_end21-awlink_asm_rx
                                        // -- End function
	.p2align	2                               // -- Begin function junk_fun17
	.type	junk_fun17,@function
junk_fun17:                             // @junk_fun17
// %bb.0:
	hint	#25
	sub	sp, sp, #16
	mov	w8, wzr
	str	w8, [sp, #8]
	mov	w9, #14
	str	w9, [sp, #8]
	str	w8, [sp, #4]
	mov	w8, #13
	str	w8, [sp, #4]
	ldr	w8, [sp, #8]
	add	w9, w8, #1
	str	w9, [sp, #8]
	tbnz	w8, #3, .LBB22_2
	b	.LBB22_1
.LBB22_1:
	ldr	w9, [sp, #8]
	ldr	w8, [sp, #4]
	mul	w8, w8, w9
	str	w8, [sp, #4]
	b	.LBB22_2
.LBB22_2:
	ldr	w8, [sp, #8]
	subs	w8, w8, #16
	b.ne	.LBB22_4
	b	.LBB22_3
.LBB22_3:
	ldr	w8, [sp, #4]
	str	w8, [sp, #12]
	b	.LBB22_5
.LBB22_4:
	ldr	w8, [sp, #8]
	str	w8, [sp, #12]
	b	.LBB22_5
.LBB22_5:
	ldr	w0, [sp, #12]
	add	sp, sp, #16
	hint	#29
	ret
.Lfunc_end22:
	.size	junk_fun17, .Lfunc_end22-junk_fun17
                                        // -- End function
	.globl	awlink_asm_probe                // -- Begin function awlink_asm_probe
	.p2align	2
	.type	awlink_asm_probe,@function
awlink_asm_probe:                       // @awlink_asm_probe
// %bb.0:
	hint	#25
	sub	sp, sp, #256
	str	x30, [x18], #8
	stp	x29, x30, [sp, #240]            // 16-byte Folded Spill
	add	x29, sp, #240
	stur	x0, [x29, #-16]
	stur	x1, [x29, #-24]
	mov	x7, xzr
	str	x7, [sp, #88]                   // 8-byte Folded Spill
	stur	x7, [x29, #-32]
	stur	x7, [x29, #-40]
	str	x7, [sp, #192]
	str	x7, [sp, #184]
	str	x7, [sp, #176]
	str	x7, [sp, #168]
	str	x7, [sp, #160]
	str	x7, [sp, #152]
	str	x7, [sp, #144]
	str	x7, [sp, #136]
	str	x7, [sp, #128]
	str	x7, [sp, #120]
	mov	w8, wzr
	str	w8, [sp, #116]
	mov	w9, #-1
	str	w9, [sp, #116]
	str	w8, [sp, #112]
	mov	w9, #35
	str	w9, [sp, #112]
	str	w8, [sp, #108]
	mov	w8, #1
	str	w8, [sp, #108]
	adrp	x8, res
	add	x8, x8, :lo12:res
	mov	x9, #-1
	str	x9, [x8]
	adrp	x9, res
	str	x9, [sp, #96]                   // 8-byte Folded Spill
	ldur	x2, [x29, #-24]
	mov	x9, sp
	str	x7, [x9, #8]
	str	x8, [x9]
	mov	w8, #160
	movk	w8, #49152, lsl #16
	mov	w0, w8
	mov	w8, #-65534
	mov	w1, w8
	mov	x3, x7
	mov	x4, x7
	mov	x5, x7
	mov	x6, x7
	bl	__arm_smccc_smc
	ldr	x8, [sp, #96]                   // 8-byte Folded Reload
	ldr	x8, [x8, :lo12:res]
	cbnz	x8, .LBB23_12
	b	.LBB23_1
.LBB23_1:
	ldur	x0, [x29, #-16]
	adrp	x1, .L.str
	add	x1, x1, :lo12:.L.str
	mov	x2, xzr
	bl	of_get_property
	stur	x0, [x29, #-40]
	ldur	x0, [x29, #-16]
	bl	of_irq_find_parent
	stur	x0, [x29, #-32]
	ldur	x8, [x29, #-32]
	cbnz	x8, .LBB23_3
	b	.LBB23_2
.LBB23_2:
	mov	w8, #-1
	stur	w8, [x29, #-4]
	b	.LBB23_13
.LBB23_3:
	ldur	x8, [x29, #-32]
	str	x8, [sp, #120]
	mov	w8, #3
	str	w8, [sp, #128]
	add	x8, sp, #120
	add	x8, x8, #12
	str	x8, [sp, #80]                   // 8-byte Folded Spill
	mov	w8, #1
	cbnz	w8, .LBB23_5
	b	.LBB23_4
.LBB23_4:
	brk	#0x5512
.LBB23_5:
	ldr	x9, [sp, #80]                   // 8-byte Folded Reload
	mov	w8, wzr
	str	w8, [x9]
	ldr	w9, [sp, #108]
	ldr	w8, [sp, #112]
	add	w8, w8, w9
	str	w8, [sp, #112]
	bl	junk_fun3
	ldur	x8, [x29, #-24]
	add	x8, x8, x8, lsl #6
	lsl	x8, x8, #1
	stur	x8, [x29, #-24]
	ldur	w9, [x29, #-24]
	ldr	w8, [sp, #112]
	add	w8, w8, w9
	str	w8, [sp, #112]
	ldr	w8, [sp, #112]
	str	w8, [sp, #60]                   // 4-byte Folded Spill
	add	x8, sp, #120
	add	x8, x8, #12
	str	x8, [sp, #64]                   // 8-byte Folded Spill
	ldrsw	x8, [sp, #108]
	mov	w0, w8
	str	w0, [sp, #76]                   // 4-byte Folded Spill
	subs	x8, x8, #16
	b.lo	.LBB23_7
	b	.LBB23_6
.LBB23_6:
	brk	#0x5512
.LBB23_7:
	ldr	x9, [sp, #64]                   // 8-byte Folded Reload
	ldr	w0, [sp, #76]                   // 4-byte Folded Reload
                                        // implicit-def: $x8
	mov	w8, w0
	sbfiz	x10, x8, #2, #32
	add	x8, x10, #12
	add	x9, x9, x10
	str	x9, [sp, #40]                   // 8-byte Folded Spill
	mov	w9, #68
                                        // kill: def $x9 killed $w9
	subs	x9, x9, x10
	str	x9, [sp, #48]                   // 8-byte Folded Spill
	subs	x8, x8, #80
	b.hi	.LBB23_14
	b	.LBB23_8
.LBB23_8:
	ldr	x8, [sp, #48]                   // 8-byte Folded Reload
	subs	x8, x8, #4
	b.lo	.LBB23_14
	b	.LBB23_9
.LBB23_9:
	ldr	w8, [sp, #60]                   // 4-byte Folded Reload
	ldr	x9, [sp, #40]                   // 8-byte Folded Reload
	str	w8, [x9]
	bl	junk_fun15
	add	x8, sp, #120
	add	x8, x8, #12
	str	x8, [sp, #32]                   // 8-byte Folded Spill
	mov	w8, #1
	cbnz	w8, .LBB23_11
	b	.LBB23_10
.LBB23_10:
	brk	#0x5512
.LBB23_11:
	ldr	x9, [sp, #32]                   // 8-byte Folded Reload
	mov	w8, #4
	str	w8, [x9, #8]
	ldur	x0, [x29, #-40]
	add	x1, sp, #120
	str	x1, [sp, #24]                   // 8-byte Folded Spill
	bl	of_irq_parse_raw
	bl	junk_fun16
                                        // kill: def $w1 killed $w0
	ldr	x0, [sp, #24]                   // 8-byte Folded Reload
	bl	irq_create_of_mapping
	str	w0, [sp, #116]
	b	.LBB23_12
.LBB23_12:
	ldr	w8, [sp, #116]
	stur	w8, [x29, #-4]
	b	.LBB23_13
.LBB23_13:
	ldur	w0, [x29, #-4]
	ldp	x29, x30, [sp, #240]            // 16-byte Folded Reload
	ldr	x30, [x18, #-8]!
	add	sp, sp, #256
	hint	#29
	ret
.LBB23_14:
	brk	#0x1
.Lfunc_end23:
	.size	awlink_asm_probe, .Lfunc_end23-awlink_asm_probe
                                        // -- End function
	.p2align	2                               // -- Begin function junk_fun3
	.type	junk_fun3,@function
junk_fun3:                              // @junk_fun3
// %bb.0:
	hint	#25
	sub	sp, sp, #16
	mov	w8, wzr
	str	w8, [sp, #12]
	mov	w9, #11
	str	w9, [sp, #12]
	str	w8, [sp, #8]
	mov	w8, #5
	str	w8, [sp, #8]
	ldr	w8, [sp, #8]
	and	w8, w8, #0x1
	subs	w8, w8, #768
	b.eq	.LBB24_2
	b	.LBB24_1
.LBB24_1:
	ldr	w9, [sp, #12]
	ldr	w8, [sp, #8]
	add	w8, w8, w9
	str	w8, [sp, #8]
	b	.LBB24_2
.LBB24_2:
	ldr	w0, [sp, #12]
	add	sp, sp, #16
	hint	#29
	ret
.Lfunc_end24:
	.size	junk_fun3, .Lfunc_end24-junk_fun3
                                        // -- End function
	.p2align	2                               // -- Begin function junk_fun15
	.type	junk_fun15,@function
junk_fun15:                             // @junk_fun15
// %bb.0:
	hint	#25
	sub	sp, sp, #16
	mov	w8, wzr
	str	w8, [sp, #8]
	mov	w9, #12
	str	w9, [sp, #8]
	str	w8, [sp, #4]
	mov	w8, #7
	str	w8, [sp, #4]
	ldr	w8, [sp, #8]
	add	w8, w8, #1
	str	w8, [sp, #8]
	ldr	w8, [sp, #4]
	add	w8, w8, #51
	str	w8, [sp, #4]
	ldr	w8, [sp, #4]
	subs	w8, w8, #7
	b.ne	.LBB25_2
	b	.LBB25_1
.LBB25_1:
	ldr	w8, [sp, #4]
	str	w8, [sp, #12]
	b	.LBB25_7
.LBB25_2:
	ldr	w8, [sp, #4]
	mov	w9, #819
	mul	w8, w8, w9
	str	w8, [sp, #8]
	ldr	w8, [sp, #8]
	add	w8, w8, #1
	str	w8, [sp, #8]
	ldr	w8, [sp, #4]
	add	w8, w8, #51
	str	w8, [sp, #4]
	ldr	w8, [sp, #4]
	subs	w8, w8, #7
	b.ne	.LBB25_4
	b	.LBB25_3
.LBB25_3:
	ldr	w8, [sp, #4]
	str	w8, [sp, #12]
	b	.LBB25_7
.LBB25_4:
	ldr	w9, [sp, #4]
	ldr	w8, [sp, #8]
	add	w8, w8, w9
	str	w8, [sp, #8]
	ldr	w8, [sp, #8]
	subs	w8, w8, #7
	b.ne	.LBB25_6
	b	.LBB25_5
.LBB25_5:
	ldr	w8, [sp, #4]
	str	w8, [sp, #12]
	b	.LBB25_7
.LBB25_6:
	ldr	w8, [sp, #8]
	str	w8, [sp, #12]
	b	.LBB25_7
.LBB25_7:
	ldr	w0, [sp, #12]
	add	sp, sp, #16
	hint	#29
	ret
.Lfunc_end25:
	.size	junk_fun15, .Lfunc_end25-junk_fun15
                                        // -- End function
	.p2align	2                               // -- Begin function junk_fun16
	.type	junk_fun16,@function
junk_fun16:                             // @junk_fun16
// %bb.0:
	hint	#25
	sub	sp, sp, #16
	mov	w8, wzr
	str	w8, [sp, #8]
	mov	w9, #9
	str	w9, [sp, #8]
	str	w8, [sp, #4]
	mov	w8, #6
	str	w8, [sp, #4]
	ldr	w8, [sp, #8]
	add	w9, w8, #1
	str	w9, [sp, #8]
	subs	w8, w8, #7
	b.ne	.LBB26_2
	b	.LBB26_1
.LBB26_1:
	mov	w8, #51
	str	w8, [sp, #4]
	b	.LBB26_2
.LBB26_2:
	ldr	w8, [sp, #8]
	and	w8, w8, #0x2
	str	w8, [sp, #4]
	ldr	w8, [sp, #8]
	subs	w8, w8, #16
	b.ne	.LBB26_4
	b	.LBB26_3
.LBB26_3:
	ldr	w8, [sp, #4]
	str	w8, [sp, #12]
	b	.LBB26_7
.LBB26_4:
	ldr	w8, [sp, #8]
	subs	w8, w8, #7
	b.ne	.LBB26_6
	b	.LBB26_5
.LBB26_5:
	ldr	w8, [sp, #4]
	str	w8, [sp, #12]
	b	.LBB26_7
.LBB26_6:
	ldr	w8, [sp, #8]
	str	w8, [sp, #12]
	b	.LBB26_7
.LBB26_7:
	ldr	w0, [sp, #12]
	add	sp, sp, #16
	hint	#29
	ret
.Lfunc_end26:
	.size	junk_fun16, .Lfunc_end26-junk_fun16
                                        // -- End function
	.p2align	2                               // -- Begin function log_write_mmio
	.type	log_write_mmio,@function
log_write_mmio:                         // @log_write_mmio
// %bb.0:
	hint	#25
	sub	sp, sp, #32
	str	x0, [sp, #24]
	strb	w1, [sp, #20]
	str	x2, [sp, #8]
	str	x3, [sp]
	add	sp, sp, #32
	hint	#29
	ret
.Lfunc_end27:
	.size	log_write_mmio, .Lfunc_end27-log_write_mmio
                                        // -- End function
	.p2align	2                               // -- Begin function log_post_write_mmio
	.type	log_post_write_mmio,@function
log_post_write_mmio:                    // @log_post_write_mmio
// %bb.0:
	hint	#25
	sub	sp, sp, #32
	str	x0, [sp, #24]
	strb	w1, [sp, #20]
	str	x2, [sp, #8]
	str	x3, [sp]
	add	sp, sp, #32
	hint	#29
	ret
.Lfunc_end28:
	.size	log_post_write_mmio, .Lfunc_end28-log_post_write_mmio
                                        // -- End function
	.p2align	2                               // -- Begin function log_read_mmio
	.type	log_read_mmio,@function
log_read_mmio:                          // @log_read_mmio
// %bb.0:
	hint	#25
	sub	sp, sp, #32
	strb	w0, [sp, #28]
	str	x1, [sp, #16]
	str	x2, [sp, #8]
	add	sp, sp, #32
	hint	#29
	ret
.Lfunc_end29:
	.size	log_read_mmio, .Lfunc_end29-log_read_mmio
                                        // -- End function
	.p2align	2                               // -- Begin function log_post_read_mmio
	.type	log_post_read_mmio,@function
log_post_read_mmio:                     // @log_post_read_mmio
// %bb.0:
	hint	#25
	sub	sp, sp, #32
	str	x0, [sp, #24]
	strb	w1, [sp, #20]
	str	x2, [sp, #8]
	str	x3, [sp]
	add	sp, sp, #32
	hint	#29
	ret
.Lfunc_end30:
	.size	log_post_read_mmio, .Lfunc_end30-log_post_read_mmio
                                        // -- End function
	.type	res,@object                     // @res
	.bss
	.globl	res
	.p2align	3
res:
	.zero	32
	.size	res, 32

	.type	.L.str,@object                  // @.str
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str:
	.asciz	"reg"
	.size	.L.str, 4

	.type	.L.src,@object                  // @.src
.L.src:
	.size	.L.src, 102

	.type	.L__unnamed_1,@object           // @0
	.section	.rodata,"a",@progbits
	.p2align	4
.L__unnamed_1:
	.hword	65535                           // 0xffff
	.hword	0                               // 0x0
	.asciz	"'uint32_t[16]' (aka 'unsigned int[16]')"
	.size	.L__unnamed_1, 44

	.type	.L__unnamed_2,@object           // @1
	.p2align	3
.L__unnamed_2:
	.hword	0                               // 0x0
	.hword	11                              // 0xb
	.asciz	"'int'"
	.size	.L__unnamed_2, 10

	.ident	"Android (8490178, based on r450784d) clang version 14.0.6 (https://android.googlesource.com/toolchain/llvm-project 4c603efb0cca074e9238af8b4106c30add4418f6)"
	.section	".note.GNU-stack","",@progbits
	.addrsig
	.addrsig_sym junk_fun1
	.addrsig_sym writel
	.addrsig_sym junk_fun2
	.addrsig_sym __arm_smccc_smc
	.addrsig_sym __ioremap
	.addrsig_sym junk_fun4
	.addrsig_sym junk_fun9
	.addrsig_sym junk_fun10
	.addrsig_sym junk_fun12
	.addrsig_sym readl
	.addrsig_sym junk_fun13
	.addrsig_sym junk_fun14
	.addrsig_sym junk_fun17
	.addrsig_sym of_get_property
	.addrsig_sym of_irq_find_parent
	.addrsig_sym junk_fun3
	.addrsig_sym junk_fun15
	.addrsig_sym of_irq_parse_raw
	.addrsig_sym junk_fun16
	.addrsig_sym irq_create_of_mapping
	.addrsig_sym log_write_mmio
	.addrsig_sym log_post_write_mmio
	.addrsig_sym log_read_mmio
	.addrsig_sym log_post_read_mmio
	.addrsig_sym res
	.addrsig_sym arm64_use_ng_mappings
