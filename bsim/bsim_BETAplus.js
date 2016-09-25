BSim = {};

$(function() {
    var split = new SplitPane('#split-container', ['#editor-pane', '#simulation-pane']);

    $('.global-controls').append('<span style="margin-right:10px;">'+$('title').text()+'</span>');

    // initial configuration
    split.setPaneWidth(0, split.window_width());
    split.setPaneWidth(1, 0);

    // Set up the split buttons.
    $('#maximise_editor').click(function() {
        split.setPaneWidth(0, split.window_width());
        split.setPaneWidth(1, 0);
    });
    $('#split_pane').click(function() {
        var width = split.window_width();
        split.setPaneWidth(0, width/2);
        split.setPaneWidth(1, width/2);
    });
    $('#maximise_simulation').click(function() {
        split.setPaneWidth(0, 0);
        split.setPaneWidth(1, split.window_width());
    });

    split.on('resize', function(widths) {
        if(widths[1] === 0) {
            $('#maximise_editor').addClass('active').siblings().removeClass('active');
        } else if(widths[0] === 0) {
            $('#maximise_simulation').addClass('active').siblings().removeClass('active');
        } else {
            $('#split_pane').addClass('active').siblings().removeClass('active');
        }
        if(widths[0] === 0) {
            editor.blur();
        }
    });

    // Make an editor
    var editor = new Editor('#editor', 'uasm', true);
    editor.openTab('beta.uasm', 
`////////////////////////////////////////////////////////////////////////
/// 6.004 BETA Macro package -                  revised 1/25/16 SAW  ///
///  This file defines our 32-bit Beta instruction set.              ///
////////////////////////////////////////////////////////////////////////

/* Global instruction definition conventions:
 *  * DESTINATION arg is LAST
 *
 * Instruction set summary.  Notation:
 * ra, rb, rc: registers
 *         CC: 16-bit signed constant
 *      label: statement/location tag (becomes PC-relative offset)
 *
 * ADD(RA, RB, RC)      // RC <- <RA> + <RB>
 * ADDC(RA, C, RC)      // RC <- <RA> + C
 * AND(RA, RB, RC)      // RC <- <RA> & <RB>
 * ANDC(RA, C, RC)      // RC <- <RA> & C
 * MUL(RA, RB, RC)      // RC <- <RA> * <RB>
 * MULC(RA, C, RC)      // RC <- <RA> * C
 * DIV(RA, RB, RC)      // RC <- <RA> / <RB>
 * DIVC(RA, C, RC)      // RC <- <RA> / C
 * OR( RA, RB, RC)      // RC <- <RA> | <RB>
 * ORC(RA,  C, RC)      // RC <- <RA> | C
 * SHL(RA, RB, RC)      // RC <- <RA> << <RB>
 * SHLC(RA, C, RC)      // RC <- <RA> << C
 * SHR(RA, RB, RC)      // RC <- <RA> >> <RB>
 * SHRC(RA, C, RC)      // RC <- <RA> >> C
 * SRA(RA, RB, RC)      // RC <- <RA> >> <RB>
 * SRAC(RA, C, RC)      // RC <- <RA> >> C
 * SUB(RA, RB, RC)      // RC <- <RA> - <RB>
 * SUBC(RA, C, RC)      // RC <- <RA> - C
 * XOR(RA, RB, RC)      // RC <- <RA> ^ <RB>
 * XORC(RA, C, RC)      // RC <- <RA> ^ C
 * XNOR(RA, RB, RC)     // RC <- ~(<RA> ^ <RB>)
 * XNORC(RA, C, RC)     // RC <- ~(<RA> ^ C)
 *
 * CMPEQ(RA, RB, RC)    // RC <- <RA> == <RB>
 * CMPEQC(RA, C, RC)    // RC <- <RA> == C
 * CMPLE(RA, RB, RC)    // RC <- <RA> <= <RB>
 * CMPLEC(RA, C, RC)    // RC <- <RA> <= C
 * CMPLT(RA, RB, RC)    // RC <- <RA> <  <RB>
 * CMPLTC(RA, C, RC)    // RC <- <RA> <  C
 *
 *
 * BR(LABEL,RC)         // RC <- <PC>+4; PC <- LABEL (PC-relative addressing)
 * BR(LABEL)            // PC <- LABEL (PC-relative addressing)
 * BEQ(RA, LABEL, RC)   // RC <- <PC>+4; IF <RA>==0 THEN PC <- LABEL
 * BEQ(RA, LABEL)       // IF <RA>==0 THEN PC <- LABEL
 * BF(RA, LABEL, RC)    // RC <- <PC>+4; IF <RA>==0 THEN PC <- LABEL
 * BF(RA, LABEL)        // IF <RA>==0 THEN PC <- LABEL
 * BNE(RA, LABEL, RC)   // RC <- <PC>+4; IF <RA>!=0 THEN PC <- LABEL
 * BNE(RA, LABEL)       // IF <RA>!=0 THEN PC <- LABEL
 * BT(RA, LABEL, RC)    // RC <- <PC>+4; IF <RA>!=0 THEN PC <- LABEL
 * BT(RA, LABEL)        // IF <RA>!=0 THEN PC <- LABEL
 * JMP(RA, RC)          // RC <- <PC>+4; PC <- <RA> & 0xFFFC
 * JMP(RB)              // PC <- <RB> & 0xFFFC
 *
 * LD(RA, CC, RC)       // RC <- <<RA>+CC>
 * LD(CC, RC)           // RC <- <CC>
 * ST(RC, CC, RA)       // <RA>+CC <- <RC>
 * ST(RC, CC)           // CC <- <RC>
 * LDR(CC, RC)          // RC <- <CC> (PC-relative addressing)
 *
 * MOVE(RA, RC)         // RC <- <RA>
 * CMOVE(CC, RC)        // RC <- CC
 * HALT()               // STOPS SIMULATOR.
 *
 * PUSH(RA)             // (2) SP <- <SP> + 4; <<SP>-4> <- <RA>
 * POP(RA)              // (2) RA <- <<SP>-4>; SP <- <SP> - 4
 * ALLOCATE(N)          // Allocate N longwords from stack
 * DEALLOCATE(N)        // Release N longwords
 *
 * CALL(label)          // Call a subr; save PC in lp.
 * CALL(label, n)       // (2) Call subr at label with n args.
 *                      // Saves return adr in LP.
 *                      // Pops n longword args from stack.
 *
 * RTN()                // Returns to adr in <LP> (Subr return)
 * XRTN()               // Returns to adr in <IP> (Intr return)
 *
 * SHORT(val)           // Assemble val as a 16-bit datum
 * LONG(val)            // Assemble val as a 32-bit datum 
 * WORD(val)            // Assemble val as a 32-bit datum 
 *                      // (LONG and WORD are synonyms)
 * GETFRAME(F, RA)      // RA <- <<BP>+F>
 * PUTFRAME(RA, F)      // <BP>+F <- <RA>
 *
 * Calling convention:
 *      PUSH(argn-1)
 *      ...
 *      PUSH(arg0)
 *      CALL(subr, nargs)
 *      (return here with result in R0, args cleaned)
 *
 * Extra register conventions, for procedure linkage:
 * LP = 28              // Linkage register (holds return adr)
 * BP = 29              // Frame pointer (points to base of frame)
 *
 * Conventional stack frames look like:
 *      arg[N-1]
 *      ...
 *      arg[0]
 *      <saved lp>
 *      <saved bp>
 *      <other saved regs>
 *   BP-><locals>
 *       ...
 *   SP->(first unused location)
 */

////////////////////////////////////////////////////////////////////////
/// End of documentation.  Following are the actual definitions...   ///
////////////////////////////////////////////////////////////////////////

// Assemble words, little-endian:
.macro SHORT(x) x%0x100 (x>>8)%0x100 
.macro LONG(x) SHORT(x) SHORT(x >> 16)  // little-endian for Beta 
.macro WORD(x) SHORT(x) SHORT(x >> 16)  // Synonym for LONG

.macro STORAGE(NWORDS)  . = .+(4*NWORDS)// Reserve NWORDS words of RAM

// register designators
// this allows symbols like r0, etc to be used as
// operands in instructions. Note that there is no real difference
// in this assembler between register operands and small integers.

r0 = 0
r1 = 1
r2 = 2
r3 = 3
r4 = 4
r5 = 5
r6 = 6
r7 = 7
r8 = 8
r9 = 9
r10 = 10
r11 = 11
r12 = 12
r13 = 13
r14 = 14
r15 = 15
r16 = 16
r17 = 17
r18 = 18
r19 = 19
r20 = 20
r21 = 21
r22 = 22
r23 = 23
r24 = 24
r25 = 25
r26 = 26
r27 = 27
r28 = 28
r29 = 29
r30 = 30
r31 = 31

bp = 27                 // frame pointer (points to base of frame)
lp = 28                 // linkage register (holds return adr)
sp = 29                 // stack pointer (points to 1st free locn)
xp = 30                 // interrupt return pointer (lp for interrupts)


// understand upper case, too.
R0 = r0
R1 = r1
R2 = r2
R3 = r3
R4 = r4
R5 = r5
R6 = r6
R7 = r7
R8 = r8
R9 = r9
R10 = r10
R11 = r11
R12 = r12
R13 = r13
R14 = r14
R15 = r15
R16 = r16
R17 = r17
R18 = r18
R19 = r19
R20 = r20
R21 = r21
R22 = r22
R23 = r23
R24 = r24
R25 = r25
R26 = r26
R27 = r27
R28 = r28
R29 = r29
R30 = r30
R31 = r31
XP = xp
LP = lp
BP = bp
SP = sp

.macro betaop(OP,RA,RB,RC) {
          .align 4
          WORD((OP<<26)+((RC%0x20)<<21)+((RA%0x20)<<16)+((RB%0x20)<<11)) }

.macro betaopc(OP,RA,CC,RC) {
          .align 4
          WORD((OP<<26)+((RC%0x20)<<21)+((RA%0x20)<<16)+(CC%0x10000)) }


.macro ADD(RA, RB, RC)          betaop(0x20,RA,RB,RC)
.macro ADDC(RA, C, RC)          betaopc(0x30,RA,C,RC)

.macro AND(RA, RB, RC)          betaop(0x28,RA,RB,RC)
.macro ANDC(RA, C, RC)          betaopc(0x38,RA,C,RC)
.macro MUL(RA, RB, RC)          betaop(0x22,RA,RB,RC)
.macro MULC(RA, C, RC)          betaopc(0x32,RA,C,RC)
.macro DIV(RA, RB, RC)          betaop(0x23,RA,RB,RC)
.macro DIVC(RA, C, RC)          betaopc(0x33,RA,C,RC)
.macro OR( RA, RB, RC)          betaop(0x29,RA,RB,RC)
.macro ORC(RA,  C, RC)          betaopc(0x39,RA,C,RC)
.macro SHL(RA, RB, RC)          betaop(0x2C,RA,RB,RC)
.macro SHLC(RA, C, RC)          betaopc(0x3C,RA,C,RC)
.macro SHR(RA, RB, RC)          betaop(0x2D,RA,RB,RC)
.macro SHRC(RA, C, RC)          betaopc(0x3D,RA,C,RC)
.macro SRA(RA, RB, RC)          betaop(0x2E,RA,RB,RC)
.macro SRAC(RA, C, RC)          betaopc(0x3E,RA,C,RC)
.macro SUB(RA, RB, RC)          betaop(0x21,RA,RB,RC)
.macro SUBC(RA, C, RC)          betaopc(0x31,RA,C,RC)
.macro XOR(RA, RB, RC)          betaop(0x2A,RA,RB,RC)
.macro XORC(RA, C, RC)          betaopc(0x3A,RA,C,RC)
.macro XNOR(RA, RB, RC)         betaop(0x2B,RA,RB,RC)
.macro XNORC(RA, C, RC)         betaopc(0x3B,RA,C,RC)

.macro CMPEQ(RA, RB, RC)        betaop(0x24,RA,RB,RC)
.macro CMPEQC(RA, C, RC)        betaopc(0x34,RA,C,RC)
.macro CMPLE(RA, RB, RC)        betaop(0x26,RA,RB,RC)
.macro CMPLEC(RA, C, RC)        betaopc(0x36,RA,C,RC)
.macro CMPLT(RA, RB, RC)        betaop(0x25,RA,RB,RC)
.macro CMPLTC(RA, C, RC)        betaopc(0x35,RA,C,RC)

.macro BETABR(OP,RA,RC,LABEL)   betaopc(OP,RA,((LABEL-.)>>2)-1, RC)
.macro BEQ(RA, LABEL, RC)       BETABR(0x1C,RA,RC,LABEL)
.macro BEQ(RA, LABEL)           BETABR(0x1C,RA,r31,LABEL)
.macro BF(RA, LABEL, RC)        BEQ(RA,LABEL,RC)
.macro BF(RA,LABEL)             BEQ(RA,LABEL)
.macro BNE(RA, LABEL, RC)       BETABR(0x1D,RA,RC,LABEL)
.macro BNE(RA, LABEL)           BETABR(0x1D,RA,r31,LABEL)
.macro BT(RA,LABEL,RC)          BNE(RA,LABEL,RC)
.macro BT(RA,LABEL)             BNE(RA,LABEL)
.macro BR(LABEL,RC)             BEQ(r31, LABEL, RC)
.macro BR(LABEL)                BR(LABEL, r31)
.macro JMP(RA, RC)              betaopc(0x1B,RA,0,RC)
.macro JMP(RA)                  betaopc(0x1B,RA,0,r31)

.macro LD(RA, CC, RC)           betaopc(0x18,RA,CC,RC)
.macro LD(CC, RC)               betaopc(0x18,R31,CC,RC)
.macro ST(RC, CC, RA)           betaopc(0x19,RA,CC,RC)
.macro ST(RC, CC)               betaopc(0x19,R31,CC,RC)
.macro LDR(CC, RC)              BETABR(0x1F, R31, RC, CC)

.macro MOVE(RA, RC)             ADD(RA, R31, RC)
.macro CMOVE(CC, RC)            ADDC(R31, CC, RC)

.macro PUSH(RA)         ADDC(SP,4,SP)  ST(RA,-4,SP)
.macro POP(RA)          LD(SP,-4,RA)   ADDC(SP,-4,SP)

.macro CALL(label)      BR(label, LP)
                        
.macro RTN()            JMP(LP)
.macro XRTN()           JMP(XP)

// Controversial Extras
// Calling convention:
//      PUSH(argn-1)
//      ...
//      PUSH(arg0)
//      CALL(subr, nargs)
//      (return here with result in R0, args cleaned)

// Extra register conventions, for procedure linkage:
// LP = 28                      // Linkage register (holds return adr)
// BP = 29                      // Frame pointer (points to base of frame)

// Conventional stack frames look like:
//      arg[N-1]
//      ...
//      arg[0]
//      <saved lp>
//      <saved bp>
//      <other saved regs>
//   BP-><locals>
//       ...
//   SP->(first unused location)

.macro CALL(S,N) BR(S,lp) SUBC(sp, 4*N, sp)

.macro ALLOCATE(N) ADDC(sp, N*4, sp)
.macro DEALLOCATE(N) SUBC(sp, N*4, sp)

//--------------------------------------------------------
// Privileged mode instructions
//--------------------------------------------------------

.macro PRIV_OP(FNCODE)          betaopc (0x00, 0, FNCODE, 0)
.macro HALT() PRIV_OP (0)
.macro RDCHAR() PRIV_OP (1)
.macro WRCHAR() PRIV_OP (2)
.macro CYCLE()  PRIV_OP (3)
.macro TIME()   PRIV_OP (4)
.macro CLICK()  PRIV_OP (5)
.macro RANDOM() PRIV_OP (6)
.macro SEED()   PRIV_OP (7)
.macro SERVER() PRIV_OP (8)

// SVC calls; used for OS extensions

.macro SVC(code)                betaopc (0x01, 0, code, 0)

// Trap and interrupt vectors
VEC_RESET       = 0             // Reset (powerup)
VEC_II          = 4             // Illegal instruction (also SVC call)
VEC_CLK         = 8             // Clock interrupt
VEC_KBD         = 12            // Keyboard interrupt
VEC_MOUSE       = 16            // Mouse interrupt

// constant for the supervisor bit in the PC
PC_SUPERVISOR      = 0x80000000         // the bit itself
PC_MASK            = 0x7fffffff         // a mask for the rest of the PC

// The following macros save and restore all 32 registers (including R31) in
// a 32-word block of memory.  There are two versions:
//   * single-argument version, whose argument WHERE must be a location
//     in low memory (i.e., addressable via a 16-bit signed offset); and
//   * 2-argument version, which includes a constant offset and a base
//     register (as in LD and ST).
// NB: That location must be in low memory, i.e., addressable by a
// 16-bit signed offset.

.macro save_all_regs(WHERE) save_all_regs(WHERE, r31)
.macro save_all_regs(WHERE, base_reg) {
        ST(r0,WHERE,base_reg)
        ST(r1,WHERE+4,base_reg)
        ST(r2,WHERE+8,base_reg)
        ST(r3,WHERE+12,base_reg)
        ST(r4,WHERE+16,base_reg)
        ST(r5,WHERE+20,base_reg)
        ST(r6,WHERE+24,base_reg)
        ST(r7,WHERE+28,base_reg)
        ST(r8,WHERE+32,base_reg)
        ST(r9,WHERE+36,base_reg)
        ST(r10,WHERE+40,base_reg)
        ST(r11,WHERE+44,base_reg)
        ST(r12,WHERE+48,base_reg)
        ST(r13,WHERE+52,base_reg)
        ST(r14,WHERE+56,base_reg)
        ST(r15,WHERE+60,base_reg)
        ST(r16,WHERE+64,base_reg)
        ST(r17,WHERE+68,base_reg)
        ST(r18,WHERE+72,base_reg)
        ST(r19,WHERE+76,base_reg)
        ST(r20,WHERE+80,base_reg)
        ST(r21,WHERE+84,base_reg)
        ST(r22,WHERE+88,base_reg)
        ST(r23,WHERE+92,base_reg)
        ST(r24,WHERE+96,base_reg)
        ST(r25,WHERE+100,base_reg)
        ST(r26,WHERE+104,base_reg)
        ST(r27,WHERE+108,base_reg)
        ST(r28,WHERE+112,base_reg)
        ST(r29,WHERE+116,base_reg)
        ST(r30,WHERE+120,base_reg)
        ST(base_reg,WHERE+124,base_reg)
} // End of save-all-regs macro

.macro restore_all_regs(WHERE) restore_all_regs(WHERE, r31)
.macro restore_all_regs(WHERE, base_reg) {
        LD(base_reg,WHERE,r0)
        LD(base_reg,WHERE+4,r1)
        LD(base_reg,WHERE+8,r2)
        LD(base_reg,WHERE+12,r3)
        LD(base_reg,WHERE+16,r4)
        LD(base_reg,WHERE+20,r5)
        LD(base_reg,WHERE+24,r6)
        LD(base_reg,WHERE+28,r7)
        LD(base_reg,WHERE+32,r8)
        LD(base_reg,WHERE+36,r9)
        LD(base_reg,WHERE+40,r10)
        LD(base_reg,WHERE+44,r11)
        LD(base_reg,WHERE+48,r12)
        LD(base_reg,WHERE+52,r13)
        LD(base_reg,WHERE+56,r14)
        LD(base_reg,WHERE+60,r15)
        LD(base_reg,WHERE+64,r16)
        LD(base_reg,WHERE+68,r17)
        LD(base_reg,WHERE+72,r18)
        LD(base_reg,WHERE+76,r19)
        LD(base_reg,WHERE+80,r20)
        LD(base_reg,WHERE+84,r21)
        LD(base_reg,WHERE+88,r22)
        LD(base_reg,WHERE+92,r23)
        LD(base_reg,WHERE+96,r24)
        LD(base_reg,WHERE+100,r25)
        LD(base_reg,WHERE+104,r26)
        LD(base_reg,WHERE+108,r27)
        LD(base_reg,WHERE+112,r28)
        LD(base_reg,WHERE+116,r29)
        LD(base_reg,WHERE+120,r30)
} // End of restore-all-regs macro

/// Macro to extract and right-adjust a bit field from RA, and leave it
/// in RB.  The bit field M:N, where M >= N.
.macro extract_field (RA, M, N, RB) {
       SHLC(RA, 31-M, RB)       // Shift left, to mask out high bits
       SHRC(RB, 31-(M-N), RB)   // Shift right, to mask out low bits.
}

// Macro to reserve N consecutive WORDS of memory:
.macro RESERVE(N) . = .+(N*4)

`     
     ,true);
    
    
    editor.openTab('Test Beta +', 
`// Checkoff benchmarks for 6.004 Design Project

.include "beta.uasm"

. = 0

Reset:  BR(Start)

IllOp:  JMP(XP)                 // return to instruction following illop

Trap:   .breakpoint
        BR(Trap)                // loop on unexpected interrupt


. = 0x20
        LONG(tos)               // Location 0x20 contains pointer to end

Start:
        CMOVE(0, R0)            // 051123 SAW: Avoid Ra bug in LDR

        LDR(tosp,SP)
        BR(bench0,LP)           // q&d functionality test
        BR(bench1,LP)           // unsigned 16-bit divide
        BR(bench2,LP)           // list processing
        BR(bench3,LP)           // JIT emulation
        BR(bench4,LP)           // arithmetic for superscalars...
        BR(bench5,LP)           // RAM accesses
        ST(LP,sdone,R31)        // Store address of this instruction.

Done:   .breakpoint
        BR(Done)

tosp:   LONG(tos)

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////
////    Benchmark #0
////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Test all the instructions to make sure none of them have disappeared :)
// We don't require MUL or DIV.

// Here's how mismatches in memory locations correspond to possible bugs.
// The verification will report the memory address as a *word* address
// Memory addr    Test
//  0x14A         no test, expect 0
//  0x14B         (test opcode 0b000001)  should be an ILLOP, expect PC+4 = 0x80000060
//  0x14C         (test opcode 0b000010)  should be an ILLOP, expect PC+4 = 0x80000068
//  0x14D         (test opcode 0b000011)  should be an ILLOP, expect PC+4 = 0x80000070
//  0x14E         (test opcode 0b000100)  should be an ILLOP, expect PC+4 = 0x80000078
//  0x14F         (test opcode 0b000101)  should be an ILLOP, expect PC+4 = 0x80000080
//  0x150         (test opcode 0b000110)  should be an ILLOP, expect PC+4 = 0x80000088
//  0x151         (test opcode 0b000111)  should be an ILLOP, expect PC+4 = 0x80000090

//  0x152         (test opcode 0b001000)  should be an ILLOP, expect PC+4 = 0x80000098
//  0x153         (test opcode 0b001001)  should be an ILLOP, expect PC+4 = 0x800000A0
//  0x154         (test opcode 0b001010)  should be an ILLOP, expect PC+4 = 0x800000A8
//  0x155         (test opcode 0b001011)  should be an ILLOP, expect PC+4 = 0x800000B0
//  0x156         (test opcode 0b001100)  should be an ILLOP, expect PC+4 = 0x800000B8
//  0x157         (test opcode 0b001101)  should be an ILLOP, expect PC+4 = 0x800000C0
//  0x158         (test opcode 0b001110)  should be an ILLOP, expect PC+4 = 0x800000C8
//  0x159         (test opcode 0b001111)  should be an ILLOP, expect PC+4 = 0x800000D0

//  0x15A         (test opcode 0b010000)  should be an ILLOP, expect PC+4 = 0x800000D8
//  0x15B         (test opcode 0b010001)  should be an ILLOP, expect PC+4 = 0x800000E0
//  0x15C         (test opcode 0b010010)  should be an ILLOP, expect PC+4 = 0x800000E8
//  0x15D         (test opcode 0b010011)  should be an ILLOP, expect PC+4 = 0x800000F0
//  0x15E         (test opcode 0b010100)  should be an ILLOP, expect PC+4 = 0x800000F8
//  0x15F         (test opcode 0b010101)  should be an ILLOP, expect PC+4 = 0x80000100
//  0x160         (test opcode 0b010110)  should be an ILLOP, expect PC+4 = 0x80000108
//  0x161         (test opcode 0b010111)  should be an ILLOP, expect PC+4 = 0x80000110

//  0x162         (test opcode 0b011000)  LD, expect Mem[0] = 0x73FF0008
//  0x163         no test, expect 0x00000000
//  0x164         (test opcode 0b011010)  should be an ILLOP, expect PC+4 = 0x80000124
//  0x165         (test opcode 0b011011)  JMP, expect PC+4 = 0x00000134  (user mode!)
//  0x166         (test opcode 0b011100)  BEQ, expect PC+4 = 0x00000140
//  0x167         (test opcode 0b011101)  BNE, expect PC+4 = 0x00000148
//  0x168         (test opcode 0b011110)  should be an ILLOP, expect PC+4 = 0x00000154
//  0x169         (test opcode 0b011111)  LDR, expect 0xDEADBEEF

//  0x16A         (test opcode 0b100000)  ADD, expect 0xDEADBEEF + 0x73FF0008 = 0x52ACBEF7
//  0x16B         (test opcode 0b100001)  SUB, expect 0xDEADBEEF - 0x73FF0008 = 0x6AAEBEE7
//  0x16C         no test, expect 0x00000000
//  0x16D         no test, expect 0x00000000
//  0x16E         (test opcode 0b100100)  two CMPEQ, add results, expect 0x00000001
//  0x16F         (test opcode 0b100101)  two CMPLT, add results, expect 0x00000001
//  0x170         (test opcode 0b100110)  two CMPLE, add results, expect 0x00000001
//  0x171         (test opcode 0b100111)  should be an ILLOP, expect PC+4 = 0x000001A0

//  0x172         (test opcode 0b101000)  AND, expect 0xDEADBEEF & 0x73FF0008 = 0x52AD0008
//  0x173         (test opcode 0b101000)  OR, expect 0xDEADBEEF | 0x73FF0008 = 0xFFFFBEEF
//  0x174         (test opcode 0b101010)  XOR, expect 0xDEADBEEF ^ 0x73FF0008 = 0xAD52BEE7
//  0x175         (test opcode 0b101011)  XNOR, expect ~(0xDEADBEEF ^ 0x73FF0008) = 0x52AD4118
//  0x176         (test opcode 0b101100)  SHL, expect 0x73FF0008 << 4 = 0x3FF00080
//  0x177         (test opcode 0b101101)  SHR, expect 0x73FF0008 >> 4 = 0x073FF000
//  0x178         (test opcode 0b101110)  SRA, expect 0x73FF0008 >> 4 = 0x073FF000
//  0x179         (test opcode 0b101111)  should be an ILLOP, expect PC+4 = 0x000001E4

//  0x17A         (test opcode 0b110000)  ADDC, expect -1 + 1 = 0x00000000
//  0x17B         (test opcode 0b110001)  SUBC, expect 0xDEADBEEF - (-1) = 0xDEADBEF0
//  0x17C         no test, expect 0x00000000
//  0x17D         no test, expect 0x00000000
//  0x17E         (test opcode 0b110100)  CMPEQC, expect 0x00000001
//  0x17F         (test opcode 0b110101)  CMPLTC, expect 0x00000001
//  0x180         (test opcode 0b110110)  CMPLEC, expect 0x00000001
//  0x181         (test opcode 0b110111)  should be an ILLOP, expect PC+4 = 0x00000214

//  0x182         (test opcode 0b111000)  ANDC, expect 0xDEADBEEF & 0x7654 = 0x00003644
//  0x183         (test opcode 0b111000)  ORC, expect 0xDEADBEEF & 0xFFFFFFFF = 0xFFFFFFFF
//  0x184         (test opcode 0b111010)  XORC, expect 0xDEADBEEF & 0xFFFFFFFF = 0x21524110
//  0x185         (test opcode 0b111011)  XNORC, expect 0xDEADBEEF & 0xFFFFFFFF = 0xDEADBEEF
//  0x186         (test opcode 0b111100)  SHLC, expect 0x73FF8000 << 32 = 0x73FF0008
//  0x187         (test opcode 0b111101)  SHRC, expect 0x73FF8000 >> 6 = 0X037AB6FB
//  0x188         (test opcode 0b111110)  SRAC, expect 0x73FF8000 >> 7  = 0xFFBD5B7D
//  0x189         (test opcode 0b111111)  should be an ILLOP, expect PC+4 = 0x00000254

//  0x190         expect 0xEDEDEDED, overwritten if JMP didn't JMP or BNE didn't branch

bench0:
        PUSH(LP)        // ADDC, ST
        CMOVE(sbench0,LP)   // pointer to where to store results

        // test all opcodes!
        // oops, skip magic built-in opcode 0
        LONG(0b000001<<26) ST(XP, 4,LP)   // illop => store PC+4 as "result"
        LONG(0b000010<<26) ST(XP, 8,LP)   // illop => store PC+4 as "result"
        LONG(0b000011<<26) ST(XP,12,LP)   // illop => store PC+4 as "result"
        LONG(0b000100<<26) ST(XP,16,LP)   // illop => store PC+4 as "result"
        LONG(0b000101<<26) ST(XP,20,LP)   // illop => store PC+4 as "result"
        LONG(0b000110<<26) ST(XP,24,LP)   // illop => store PC+4 as "result"
        LONG(0b000111<<26) ST(XP,28,LP)   // illop => store PC+4 as "result"

        LONG(0b001000<<26) ST(XP,32,LP)   // illop => store PC+4 as "result"
        LONG(0b001001<<26) ST(XP,36,LP)   // illop => store PC+4 as "result"
        LONG(0b001010<<26) ST(XP,40,LP)   // illop => store PC+4 as "result"
        LONG(0b001011<<26) ST(XP,44,LP)   // illop => store PC+4 as "result"
        LONG(0b001100<<26) ST(XP,48,LP)   // illop => store PC+4 as "result"
        LONG(0b001101<<26) ST(XP,52,LP)   // illop => store PC+4 as "result"
        LONG(0b001110<<26) ST(XP,56,LP)   // illop => store PC+4 as "result"
        LONG(0b001111<<26) ST(XP,60,LP)   // illop => store PC+4 as "result"

        LONG(0b010000<<26) ST(XP,64,LP)   // illop => store PC+4 as "result"
        LONG(0b010001<<26) ST(XP,68,LP)   // illop => store PC+4 as "result"
        LONG(0b010010<<26) ST(XP,72,LP)   // illop => store PC+4 as "result"
        LONG(0b010011<<26) ST(XP,76,LP)   // illop => store PC+4 as "result"
        LONG(0b010100<<26) ST(XP,80,LP)   // illop => store PC+4 as "result"
        LONG(0b010101<<26) ST(XP,84,LP)   // illop => store PC+4 as "result"
        LONG(0b010110<<26) ST(XP,88,LP)   // illop => store PC+4 as "result"
        LONG(0b010111<<26) ST(XP,92,LP)   // illop => store PC+4 as "result"

        CMOVE(-1,R0)
        LD(R0,1,R23) ST(R23,96,LP)  // LD location 0
        // we've been testing ST all along...
        LONG(0b011010<<26) ST(XP,104,LP)   // illop => store PC+4 as "result"
        CMOVE(b0_jmp,R17) JMP(R17) ST(XP,sbench0_bad,R31)
b0_jmp: ST(R17,108,LP)  // JMP
        LDR(b0_consts+4,R13) BEQ(R13,.+4,XP) ST(XP,112,LP) // BEQ
        BNE(R13,.+8,XP) ST(LP,sbench0_bad,R31) ST(XP,116,LP) // BNE
        LONG(0b011110<<26) ST(XP,120,LP)   // illop => store PC+4 as "result"
        ST(R13,124,LP)  // LDR

        ADD(R13,R23,XP) ST(XP,128,LP)  // ADD
        SUB(R13,R23,XP) ST(XP,132,LP)  // SUB
        // skip MUL and DIV
        CMPEQ(R13,R13,R2) CMPEQ(R13,R23,R3) ADD(R2,R3,XP) ST(XP,144,LP)  // CMPEQ
        CMPLT(R13,R13,R2) CMPLT(R13,R23,R3) ADD(R2,R3,XP) ST(XP,148,LP)  // CMPLT
        CMPLE(R13,R13,R2) CMPLE(R23,R13,R3) ADD(R2,R3,XP) ST(XP,152,LP)  // CMPLE
        LONG(0b100111<<26) ST(XP,156,LP)   // illop => store PC+4 as "result"

        AND(R13,R23,XP) ST(XP,160,LP)
        OR(R13,R23,XP) ST(XP,164,LP)
        XOR(R13,R23,XP) ST(XP,168,LP)
        XNOR(R13,R23,XP) ST(XP,172,LP)
        CMOVE(4,R2) SHL(R23,R2,XP) ST(XP,176,LP)
        SHR(R23,R2,XP) ST(XP,180,LP)
        SRA(R23,R2,XP) ST(XP,184,LP)
        LONG(0b101111<<26) ST(XP,188,LP)   // illop => store PC+4 as "result"

        ADDC(R0,0x1,XP) ST(XP,192,LP)      // worst case?  -1 + 1
        SUBC(R13,0xFFFF,XP) ST(XP,196,LP)
        // skip MULC, DIVC
        CMPEQC(LP,sbench0,XP) ST(XP,208,LP)
        CMPLTC(R13,0,XP) ST(XP,212,LP)
        CMPLEC(R0,-1,XP) ST(XP,216,LP)      // worst case? -1 - -1, then compare
        LONG(0b110111<<26) ST(XP,220,LP)   // illop => store PC+4 as "result"
        
        ANDC(R13,0x7654,XP) ST(XP,224,LP)
        ORC(R13,0xFFFF,XP) ST(XP,228,LP)
        XORC(R13,0xFFFF,XP) ST(XP,232,LP)
        XNORC(R13,0xFFFF,R13) ST(R13,236,LP)
        SHLC(R23,32,XP) ST(XP,240,LP)
        SHRC(R13,6,XP) ST(XP,244,LP)
        SRAC(R13,7,XP) ST(XP,248,LP)
        LONG(0b111111<<26) ST(XP,252,LP)   // illop => store PC+4 as "result"

        POP(LP)
        JMP(LP)

b0_consts:
        LONG(0xABADBABE)
        LONG(0xDEADBEEF)

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////
////    Benchmark #1
////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// This benchmark makes two calls to an subroutine
// that performs an unsigned divide of its arguments,
// and then checks the returned results.

bench1:
        PUSH(LP)
        CMOVE(32761,R0)
        CMOVE(5,R1)
        BR(UDiv16,LP)
        ST(R1,sbench1,R31)      // quotient: should be 6552 = 0x1998
        ST(R0,sbench1+4,R31)    // remainder: should be 1

        CMOVE(32227,R0)
        CMOVE(37,R1)
        BR(UDiv16,LP)
        ST(R1,sbench1+8,R31)    // quotient: should be 871 = 0x367
        ST(R0,sbench1+12,R31)   // remainder: should be 0

        POP(LP)
        JMP(LP)

// unsigned 16-bit divide: R0 = dividend,  R1 = divisor
// results: R0 = remainder, R1 = quotient

UDiv16:
        CMOVE(16,R2)            // W = 16
        CMOVE(0,R3)             // Q = 0

Uloop:  SHL(R1,R2,R4)           // r4 = divisor << iter
        CMOVE(1,R5)
        SHL(R5,R2,R5)           // r5 = 1 << iter
        CMPLTC(R0,0,R6)
        BT(R6,Uneg)
        SUB(R0,R4,R0)           // R -= divisor << iter
        ADD(R3,R5,R3)           // Q += 1 << iter
        BR(Unext)

Uneg:   ADD(R0,R4,R0)           // R += divisor << iter
        SUB(R3,R5,R3)           // Q -= 1 << iter

Unext:  SUBC(R2,1,R2)           // iter -= 1
        CMPLTC(R2,0,R6)
        BF(R6,Uloop)

        CMPLTC(R0,0,R6)
        BF(R6,Udone)
        ADD(R1,R0,R0)
        SUBC(R3,1,R3)

Udone:  MOVE(R3,R1)             // R0 = remainder, R1 = quotient
        JMP(LP)

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////
////    Benchmark #2
////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// This benchmark makes two subroutine calls: one
// to do an in-place reverse an 11-element list,
// the second to compare the reversed list with a
// "answer" list to see if all went well.

bench2: PUSH(LP)
        CMOVE(list1,R0)
        PUSH(R0)
        BR(Reverse,LP)
        DEALLOCATE(1)
        PUSH(R0)
        CMOVE(list2,R0)
        PUSH(R0)
        BR(Equal,LP)
        DEALLOCATE(2)
        ST(R0,sbench2,R31)      // should be 1
        POP(LP)
        JMP(LP)

// in-place reverse of list
Reverse:
        PUSH(LP)
        PUSH(BP)
        MOVE(SP,BP)
        PUSH(R1)
        PUSH(R2)
        CMOVE(0,R0)     // last = nil
        LD(BP,-12,R1)
        BEQ(R1,Rdone)
Rloop:
        LD(R1,4,R2)     // next = cdr(this)
        ST(R0,4,R1)
        MOVE(R1,R0)     // last = this
        MOVE(R2,R1)     // this = next
        BNE(R1,Rloop)
Rdone:                  // return last
        POP(R2)
        POP(R1)
        POP(BP)
        POP(LP)
        JMP(LP)

// see if two lists are equal
Equal:
        PUSH(LP)
        PUSH(BP)
        MOVE(SP,BP)
        PUSH(R1)
        PUSH(R2)
        PUSH(R3)
        LD(BP,-12,R0)
        LD(BP,-16,R1)
Eloop:
        BNE(R0,E1)      // if (null R0) and (null R1) return t
        BEQ(R1,Etrue)
E1:
        BEQ(R1,Efalse)
        LD(R0,0,R2)
        LD(R1,0,R3)
        CMPEQ(R2,R3,R2)
        BF(R2,Efalse)
        LD(R0,4,R0)
        LD(R1,4,R1)
        BR(Eloop)
Efalse:
        CMOVE(0,R0)
        BR(Edone)
Etrue:
        CMOVE(1,R0)     // return t
Edone:  
        POP(R3)
        POP(R2)
        POP(R1)
        POP(BP)
        POP(LP)
        JMP(LP)

list2:  LONG(1)
        LONG(list2a)
list2i: LONG(10)
        LONG(list2j)
list2b: LONG(3)
        LONG(list2c)
list2h: LONG(9)
        LONG(list2i)
list2d: LONG(5)
        LONG(list2e)
list2f: LONG(7)
        LONG(list2g)
list2e: LONG(6)
        LONG(list2f)
list2c: LONG(4)
        LONG(list2d)
list2g: LONG(8)
        LONG(list2h)
list2a: LONG(2)
        LONG(list2b)
list2j: LONG(11)
        LONG(0)

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////
////    Benchmark #3
////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// This benchmark makes a copy of itself 0x200
// bytes further up in memory and then jumps
// to the first location of the copy.  The
// process is repeated 2 times.

bench3: PUSH(LP)
        CMOVE(2,R2)
        CMOVE(sbench3,R7)

b3Start:
        BR(.+4,LP)
        CMOVE(b3End-b3Start,R0)
        MOVE(R7,R6)

b3Loop: LD(LP,-4,R1)
        ST(R1,0,R7)
        ADDC(LP,4,LP)
        ADDC(R7,4,R7)
        SUBC(R0,4,R0)
        BNE(R0,b3Loop)
        SUBC(R2,1,R2)
        BEQ(R2,b3Done)
        JMP(R6)                 // also enters user mode!

b3Done: POP(LP)
        JMP(LP)
b3End:

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////
////    Benchmark #4
////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// This benchmark just performs a lot of register arithmetic
// and writes the result to memory -- should be a slam
// dunk for pipelined and superscalar machines.

bench4: CMOVE(0,R0)             // initialize accumulators
        CMOVE(0,R1)
        CMOVE(20,R2)            // loop counter
b4loop: ADDC(R0,1,R0)
        ADDC(R1,3,R1)
        ADD(R0,R1,R0)
        ADD(R0,R1,R1)
        SUBC(R2,1,R2)
        BNE(R2,b4loop)
        ADD(R1,R0,R0)           // catch annullment problems
        ST(R0,sbench4,R31)      // should be 0x5D7BD920
        JMP(LP)


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////
////    Benchmark #5
////
//// This was put in just to make the cheap expedient of reducing main memory
//// not be sufficient to gain lots of points...
////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bench5:
        CMOVE(4096, R0)         // We'd better have 1024 words of RAM...
        ST(LP, -4, R0)
        LD(R0, -4, R0)
        ST(R0, sbench5, R31)
        JMP(LP)


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////
////    Benchmark r/w storage
////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

sbench0:                        // benchmark #1 results
        . = . + (64*4)          // one for each opcode
sbench0_bad:
        LONG(0xEDEDEDED)        // this one shouldn't be overwritten!

sbench1:                        // benchmark #1 results: 4 words
        LONG(0xEDEDEDED)        // should be overwritten...
        LONG(0xEDEDEDED)        // should be overwritten...
        LONG(0xEDEDEDED)        // should be overwritten...
        LONG(0xEDEDEDED)        // should be overwritten...

sbench2:                        // benchmark #2 results: 1 word + list
        LONG(0xEDEDEDED)

list1x: LONG(1)
        LONG(0)
list1i: LONG(10)
        LONG(list1h)
list1b: LONG(3)
        LONG(list1a)
list1h: LONG(9)
        LONG(list1g)
list1d: LONG(5)
        LONG(list1c)
list1f: LONG(7)
        LONG(list1e)
list1e: LONG(6)
        LONG(list1d)
list1c: LONG(4)
        LONG(list1b)
list1g: LONG(8)
        LONG(list1f)
list1a: LONG(2)
        LONG(list1x)
list1:  LONG(11)
        LONG(list1i)

sbench3:                        // benchmark #3 results: 30 words
        . = . + (4*30)

sbench4:                        // benchmark #4 results: 1 word
        LONG(0xEDEDEDED)        // should be overwritten...

sbench5:
        LONG(0xBADBABE)

sdone:                          // finish marker
        LONG(0xEDEDEDED)        // should be overwritten...

tos:    . = . + 0x100
        . = 0x1000

`
, true);
    $('.editor-file-control').hide();     // hide file buttons
    $('#editor .nav-tabs .close').hide();  // hide close button on tab(s)

    var do_assemble = function() {
        var filename = editor.currentTab();
        var content = editor.content('assemble');
        var metadata = editor.metadata();
        var assembler = new BetaAssembler(editor);
        editor.clearErrors();
        assembler.assemble(filename, content, metadata, function(success, result) {
            if(!success) {
                PassiveAlert("Assembly failed.", "error");
                _.each(result, function(error) {
                    if(!_.contains(editor.filenames(), error.file)) {
                        editor.openFile(error.file, true, function(editor_filename, content) {
                            editor.markErrorLine(editor_filename, error.message, error.line - 1, error.column);
                        });
                    } else {
                        editor.markErrorLine(error.file, error.message, error.line - 1, error.column);
                    }
                });
            } else {
                PassiveAlert("Assembled successfully", "success");
                beta.setSources(result.sources);
                beta.loadBytes(result.image,result.source_map);
                beta.setBreakpoints(result.breakpoints);
                beta.setLabels(result.labels);
                _.each(result.options, function(value, key) {
                    beta.setOption(key, value);
                });
                beta.getMemory().setProtectedRegions(result.protection);
                if(result.checkoff) {
                    if(result.checkoff.kind == 'tty') {
                        beta.setVerifier(new BSim.TextVerifier(beta, result.checkoff));
                    } else if(result.checkoff.kind == 'memory') {
                        beta.setVerifier(new BSim.MemoryVerifier(beta, result.checkoff));
                    }
                } else {
                    beta.setVerifier(null);
                }
                if(split.currentState()[1] === 0) {
                    $('#maximise_simulation').click();
                }
            }
        });
    };

    // Add some buttons to it
    editor.addButtonGroup([new ToolbarButton('Assemble', do_assemble, 'Runs your program!')]);

    function window_height() {
        return $('.xblock-6004').innerHeight();
    };

    var set_height = function() {
        editor.setHeight(window_height() - $('.btn-toolbar').height() - $('.nav-tabs').height()); // Set height to window height minus title.
    };
    set_height();
    $(window).resize(set_height); // Update the height whenever the browser window changes size.
    split.on('resize', _.throttle(editor.redraw, 50));

    // Stuff for the simulator
    var do_resize = function(holder, view, difference) {
        if(holder.parents('#programmer-view').length) {
            $(window).resize(function() {
                var height = window_height() - difference;
                view.resize(height);
                holder.css({height: height});
            });
        }
    };

    var beta = new BSim.Beta(80); // This starting number is basically irrelevant

    $('.regfile').each(function() {
        new BSim.RegfileView(this, beta);
    });

    $('.tty').each(function() {
        new BSim.TTY(this, beta);
    });

    $('.disassembly').each(function() {
        var view = new BSim.DisassembledView(this, beta);
        do_resize($(this), view, 470);
    });

    $('.memory').each(function() {
        var view = new BSim.MemoryView(this, beta);
        do_resize($(this), view, 272);
    });

    $('.stack').each(function() {
        var view = new BSim.StackView(this, beta);
        do_resize($(this), view, 272);
    });

    new BSim.Beta.ErrorHandler(beta);
    var schematic = new BSim.SchematicView($('svg.schematic'), beta);
    split.on('resize', BSim.SchematicView.Scale);
    $(window).resize(BSim.SchematicView.Scale);

    $('.program-controls').each(function() {
        controls = new BSim.Controls(this, beta, editor, schematic);
    });

    // Work around weird sizing bug.
    _.delay(function() {
        $(window).resize();
    }, 10);

    // // Convenient way of loading a file for testing and such.
    // var neuter = function(e) {
    //     e.stopPropagation();
    //     e.preventDefault();
    // };
    // $('body').on('dragenter', neuter);
    // $('body').on('dragover', neuter);
    // $('body').on('drop', function(e) {
    //     neuter(e);
    //     console.log(e);
    //     var dt = e.originalEvent.dataTransfer;
    //     var files = dt.files;

    //     if(files.length === 0) return;
    //     var file = files[0];
    //     beta.stop(); // Just in case.
    //     var reader = new FileReader();
    //     reader.onload = function(e) {
    //         console.log(e);
    //         //beta = new BSim.Beta(e.target.result.length);
    //         var result = new Uint8Array(e.target.result);
    //         beta.loadBytes(result);
    //         console.log("Loaded", result.length, "bytes");
    //     };
    //     reader.readAsArrayBuffer(file);
    // });

    // For debugging
    window.beta = beta;
    window.editor = editor;

    //////////////////////////////////////////////////    
    //  edX interface
    //////////////////////////////////////////////////    

    var configuration = {};  // all state saved by edX server
    var controls;

    function update_tests() {
        try {
            var checkoff = controls.get_checkoff();
            if (checkoff !== undefined) {
                // key is checksum
                configuration.tests = checkoff;
            }
        } catch(e) {
            // do nothing...
        }
    }

    // return JSON representation to be used by server-side grader
    BSim.getGrade = function () {
        update_tests();
        var grade = {'tests': configuration.tests || {}};
        if (configuration.required_tests)
          grade.required_tests = configuration.required_tests;
        return JSON.stringify(grade);
    };

    // return JSON representation of persistent state
    BSim.getState = function () {
        update_tests();

        // start with all the ancillary information
        var state = $.extend({},configuration);
        delete state.initial_state;

        // gather up contents of editor buffers
        state.state = editor.get_all_documents(true);

        return JSON.stringify(state);
    };

    // process incoming state from jsinput framework
    // This function will be called with 1 argument when JSChannel is not used,
    // 2 otherwise. In the latter case, the first argument is a transaction 
    // object that will not be used here (see http://mozilla.github.io/jschannel/docs/)
    BSim.setState = function () {
        var stateStr = arguments.length === 1 ? arguments[0] : arguments[1];
        setTimeout(function () { BSim.setStateSync(stateStr); },1);
    };

    // 6.004 uses synchronous version...
    BSim.setStateSync = function (stateStr) {
        configuration = JSON.parse(stateStr);

        // open editor tabs for each saved buffer
        editor.closeAllTabs();
        var first = true;
        $.each(configuration.initial_state || {},
               function (name,contents) {
                   editor.openTab(name,contents,false,null,true);
               });
        $.each(configuration.state || {},
               function (name,contents) {
                   editor.openTab(name,contents,first);
                   first = false;
               });

        $('.editor-file-control').hide();     // hide file buttons
        $('#editor .nav-tabs .close').hide();  // hide close button on tab(s)
    };
    
    // if we're in an iframe...
    if (window.parent !== window) {
        // make iframe resizable if we can.  This may fail if we don't have
        // access to our parent...
        try {
            // look through all our parent's iframes
            $('iframe',window.parent.document).each(function () {
                // is this iframe us?
                if (this.contentWindow == window) {
                    // yes! so add css to enable resizing
                    $(this).css({resize:'both', overflow:'auto'});

                    // initial state is JSON stored as text child of <iframe>
                    var state = JSON.parse($(this).text() || '{}');

                    // grab our server-side state from the appropriate input field
                    var id = $(this).attr('data-id');
                    if (id) {
                        var input = $("[name='"+id+"']",window.parent.document);
                        if (input) {
                            // overwrite with user's state from server
                            input = input.val();
                            if (input.length > 0) {
                                var args = JSON.parse(input);
                                args.student_id = window.parent.anonymous_student_id;
                                $.extend(state,args);
                            }
                        }
                    }

                    BSim.setStateSync(JSON.stringify(state));
                }
            });
        } catch (e) {
        }
    }

});
