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
/// 6.004 BETA Macro package -                  revised 9/16/15 CJT  ///
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
 * ADD(RA, RB, RC)      // RC ← <RA> + <RB>
 * ADDC(RA, C, RC)      // RC ← <RA> + C
 * AND(RA, RB, RC)      // RC ← <RA> & <RB>
 * ANDC(RA, C, RC)      // RC ← <RA> & C
 * MUL(RA, RB, RC)      // RC ← <RA> * <RB>
 * MULC(RA, C, RC)      // RC ← <RA> * C
 * DIV(RA, RB, RC)      // RC ← <RA> / <RB>
 * DIVC(RA, C, RC)      // RC ← <RA> / C
 * OR( RA, RB, RC)      // RC ← <RA> | <RB>
 * ORC(RA,  C, RC)      // RC ← <RA> | C
 * SHL(RA, RB, RC)      // RC ← <RA> << <RB>
 * SHLC(RA, C, RC)      // RC ← <RA> << C
 * SHR(RA, RB, RC)      // RC ← <RA> >> <RB>
 * SHRC(RA, C, RC)      // RC ← <RA> >> C
 * SRA(RA, RB, RC)      // RC ← <RA> >> <RB>
 * SRAC(RA, C, RC)      // RC ← <RA> >> C
 * SUB(RA, RB, RC)      // RC ← <RA> - <RB>
 * SUBC(RA, C, RC)      // RC ← <RA> - C
 * XOR(RA, RB, RC)      // RC ← <RA> ^ <RB>
 * XORC(RA, C, RC)      // RC ← <RA> ^ C
 * XNOR(RA, RB, RC)     // RC ← ~(<RA> ^ <RB>)
 * XNORC(RA, C, RC)     // RC ← ~(<RA> ^ C)
 *
 * CMPEQ(RA, RB, RC)    // RC ← <RA> == <RB>
 * CMPEQC(RA, C, RC)    // RC ← <RA> == C
 * CMPLE(RA, RB, RC)    // RC ← <RA> <= <RB>
 * CMPLEC(RA, C, RC)    // RC ← <RA> <= C
 * CMPLT(RA, RB, RC)    // RC ← <RA> <  <RB>
 * CMPLTC(RA, C, RC)    // RC ← <RA> <  C
 *
 *
 * BR(LABEL,RC)         // RC ← <PC>+4; PC ← LABEL (PC-relative addressing)
 * BR(LABEL)            // PC ← LABEL (PC-relative addressing)
 * BEQ(RA, LABEL, RC)   // RC ← <PC>+4; IF <RA>==0 THEN PC ← LABEL
 * BEQ(RA, LABEL)       // IF <RA>==0 THEN PC ← LABEL
 * BF(RA, LABEL, RC)    // RC ← <PC>+4; IF <RA>==0 THEN PC ← LABEL
 * BF(RA, LABEL)        // IF <RA>==0 THEN PC ← LABEL
 * BNE(RA, LABEL, RC)   // RC ← <PC>+4; IF <RA>!=0 THEN PC ← LABEL
 * BNE(RA, LABEL)       // IF <RA>!=0 THEN PC ← LABEL
 * BT(RA, LABEL, RC)    // RC ← <PC>+4; IF <RA>!=0 THEN PC ← LABEL
 * BT(RA, LABEL)        // IF <RA>!=0 THEN PC ← LABEL
 * JMP(RA, RC)          // RC ← <PC>+4; PC ← <RA> & 0xFFFC
 * JMP(RB)              // PC ← <RB> & 0xFFFC
 *
 * LD(RA, CC, RC)       // RC ← <<RA>+CC>
 * LD(CC, RC)           // RC ← <CC>
 * ST(RC, CC, RA)       // <RA>+CC ← <RC>
 * ST(RC, CC)           // CC ← <RC>
 * LDR(CC, RC)          // RC ← <CC> (PC-relative addressing)
 *
 * MOVE(RA, RC)         // RC ← <RA>
 * CMOVE(CC, RC)        // RC ← CC
 * HALT()               // STOPS SIMULATOR.
 *
 * PUSH(RA)             // (2) <SP> ← <RA>; SP ← <SP> - 4
 * POP(RA)              // (2) RA ← <<SP>+4>; SP ← <SP> + 4
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
 * WORD(val)            // Assemble val as a 16-bit datum
 * LONG(val)            // Assemble val as a 32-bit datum
 * STORAGE(NWORDS)      // Reserve NWORDS 32-bit words of DRAM
 *
 * GETFRAME(F, RA)      // RA ← <<BP>+F>
 * PUTFRAME(RA, F)      // <BP>+F ← <RA>
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
 *
 * Convention: define a symbol for each arg/local giving bp-relative offset.
 * Then use
 *   getframe(name, r) gets value at offset into register r.
 *   putframe(r, name) puts value from r into frame at offset name
 */

////////////////////////////////////////////////////////////////////////
/// End of documentation.  Following are the actual definitions...   ///
////////////////////////////////////////////////////////////////////////

// Assemble words, little-endian:
.macro WORD(x) x%0x100 (x>>8)%0x100 
.macro LONG(x) WORD(x) WORD(x >> 16)  // little-endian for Maybe
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
          LONG((OP<<26)+((RC%0x20)<<21)+((RA%0x20)<<16)+((RB%0x20)<<11)) }

.macro betaopc(OP,RA,CC,RC) {
          .align 4
          LONG((OP<<26)+((RC%0x20)<<21)+((RA%0x20)<<16)+(CC%0x10000)) }


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

// Convention: define a symbol for each arg/local giving bp-relative offset.
// Then use
//   getframe(name, r) gets value at offset into register r.
//   putframe(r, name) puts value from r into frame at offset name


.macro GETFRAME(OFFSET, REG) LD(bp, OFFSET, REG)
.macro PUTFRAME(REG, OFFSET) ST(REG, OFFSET, bp)
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
    
    
    editor.openTab('Test Beta', 
`// see if it's *really* a Beta or only just pretending...

// This code makes the following assumptions about the Beta design:
//   after reset, the Beta starts executing at location 0
//   illegal instructions cause a trap to location 4
//   interrupts cause a trap to location 8

// If this program completes successfully, it enters a two-instruction
// loop at locations 0x5EC and 0x5F0.  It reaches 0x3C4 for the first
// time on cycle 270.

// If this program detects an error, it enters a two-instruction loop at
// locations 0x00C and 0x010 with an error code in R0.  The instruction
// at 0x00C is ADDC(R0,0,R31) so it is usually possible to use the
// waveform browser in the simulator to display the error code.

// possible error codes in R0:
//  4: BEQ(R31,...) didn't branch
//  5: BNE(R31,...) did branch

//  6: CMPEQC(R31,0,R0) failed
//  7: CMPLEC(R31,0,R0) failed
//  8: CMPLTC(R31,1,R0) failed
//  9: CMPEQC(R31,-1,R0) failed
//  10: CMPLEC(R31,-1,R0) failed
//  11: CMPLTC(R31,-1,R0) failed
//  12: didn't generate 0x8000000 or 0x7FFFFFFF correctly (see code)
//  13: 0x8000000 <= 0x7FFFFFF failed
//  14: 0x7FFFFFF <= 0x80000000 failed

//  15 - 47: some op/opc instruction failed
//  48: ADDC(R31,31,R31) changed the value of R31!

//  50: JMP executed following instruction
//  51: JMP didn't fill LP correctly

//  52: LDR failed or 0xAAAAAAAA + 0x55555555 != 1
//  53: 0xAAAAAAAA + 0xAAAAAAAA != 0x55555554
//  54: 0x55555555 + 0x55555555 != 0xAAAAAAAA
//  55: 1 - 1 != 0 (carry propagation test)

//  56: 0x0F0F & 0x7F00 != 0x0F00
//  57: 0x0F0F // 0x7F00 != 0x7F0F
//  58: 0x0F0F ^ 0x7F00 != 0x700F
//  59: ~(0x0F0F ^ 0x7F00) != 0xFFFF8FF0

//  60: 1 << 32 != 1
//  61: (1 << 31) >>signed 17 != 0xFFFFC000
//  62: (1 << 31) >>unsigned 17 != 0x00004000

//  63: XP not filled correctly on illegal op trap
//  64: expected 7 illops: 08, 1A, 1E, 27, 2F, 37, 3F

//  65: load or store failure
//  66: load or store failure
//  67: load or store failure
//  68: load or store failure

//  69: alu-bypass failure
//  70: pc-bypass failure

//  71: interrupt didn't happen on correct cycle
//  72: XP not filled correctly on interrupt
//  73: ensure JMP can't go from user mode to supervisor mode
//  74: test Beta's Z logic

.include "beta.uasm"

.macro FAIL_T(RA,TEST) { BF(RA,.+12) ADDC(R31,TEST,R0) BR(Error) }
.macro FAIL_F(RA,TEST) { BT(RA,.+12) ADDC(R31,TEST,R0) BR(Error) }

. = 0

// here on reset

        BEQ(R31,Start,XP)               // should branch
error1 = .

// here on illegal instructions or if BEQ above didn't work

        BR(IllInst)

// here on interrupts

        JMP(XP)                 // return from interrupt (don't adjust XP!)

// here when we've found an error

Error:
        ADDC(R0,0,R31)                  // put error code on result bus
        BR(Error)                       // loop (or least try to!)

// handler for illegal instructions
IllInst:
        CMPEQC(XP,error1,R0)            // did the first instruction fail?
        BF(R0,Ill_1)                    // nope, do regular processing
        MOVE(XP,R0)                     // yup, branch to error handler
        BR(Error)

Ill_1:  ADDC(R1,1,R1)                   // update count
        JMP(XP)                         // return to user...

// okay, now for some real tests...
Start:
test5:
        FAIL_T(R31,5)                   // shouldn't branch

        // test simple comparisons
test6:
        CMPEQC(R31,0,R0)
        FAIL_F(R0,6)
test7:
        CMPLEC(R31,0,R0)
        FAIL_F(R0,7)
test8:
        CMPLTC(R31,1,R0)
        FAIL_F(R0,8)
test9:
        CMPEQC(R31,-1,R0)
        FAIL_T(R0,9)
test10:
        CMPLEC(R31,-1,R0)
        FAIL_T(R0,10)
test11:
        CMPLTC(R31,-1,R0)
        FAIL_T(R0,11)

        // test comparisons with overflow
test12:
        ADDC(R31,1,R1)                  // load 0x8000000 into r0
        SHLC(R1,31,R1)
        SUBC(R1,1,R2)                   // load 0x7FFFFFF into r1
        OR(R1,R2,R3)                    // make sure we have what we think we have
        CMPEQC(R3,-1,R3)
        FAIL_F(R3,12)
test13:
        CMPLE(R1,R2,R3)                 // subtraction should overflow
        FAIL_F(R3,13)
test14:
        CMPLE(R2,R1,R3)                 // test it the other way too
        FAIL_T(R3,14)

        // test the registers
test15:
        ADD(R31,R31,R0)                 // start by making sure ADD works
        FAIL_T(R0,15)
test16:
        ADDC(R31,0,R0)                  // and then ADDC
        FAIL_T(R0,16)

        // use the OP and OPC instructions to fill R1-R30 with their number
test18:
        CMPEQ(R0,R31,R1)
        CMPEQC(R1,1,R0)
        FAIL_F(R0,18)
        
test19:
        SHLC(R1,1,R2)
        CMPEQC(R2,2,R0)
        FAIL_F(R0,19)
        
test20:
        ADD(R1,R2,R3)
        CMPEQC(R3,3,R0)
        FAIL_F(R0,20)

test21:
        ADDC(R1,3,R4)
        CMPEQC(R4,4,R0)
        FAIL_F(R0,21)
        
test22:
        XORC(R31,-1,R5) XORC(R5,-6,R5)
        CMPEQC(R5,5,R0)
        FAIL_F(R0,22)
        
test23:
        OR(R4,R2,R6)
        CMPEQC(R6,6,R0)
        FAIL_F(R0,23)
        
test24:
        SUBC(R31,1,R7) SHRC(R7,29,R7)
        CMPEQC(R7,7,R0)
        FAIL_F(R0,24)
        
test25:
        SHL(R1,R3,R8)
        CMPEQC(R8,8,R0)
        FAIL_F(R0,25)

test26:
        CMPLE(R8,R8,R9) SUBC(R9,-8,R9)
        CMPEQC(R9,9,R0)
        FAIL_F(R0,26)

test27:
        XOR(R8,R2,R10)
        CMPEQC(R10,10,R0)
        FAIL_F(R0,27)

test28:
        ORC(R3,8,R11)
        CMPEQC(R11,11,R0)
        FAIL_F(R0,28)

test29:
        SUB(R31,1,R12) SHRC(R12,29,R12) ADD(R12,R5,R12)
        CMPEQC(R12,12,R0)
        FAIL_F(R0,24)

test30:
        OR(R8,R5,R13)
        CMPEQC(R13,13,R0)
        FAIL_F(R0,30)

test31:
        CMPLT(R12,R13,R14) ADD(R13,R14,R14)
        CMPEQC(R14,14,R0)
        FAIL_F(R0,31)
        
test32:
        SHLC(R1,4,R15) SUB(R15,1,R15)
        CMPEQC(R15,15,R0)
        FAIL_F(R0,32)

test33:
        SHR(R15,2,R16) ADD(R16,R13,R16)
        CMPEQC(R16,16,R0)
        FAIL_F(R0,33)

test34:
        XNORC(R16,-2,R17)
        CMPEQC(R17,17,R0)
        FAIL_F(R0,34)

test35:
        AND(R15,R2,R18) ORC(R18,16,R18)
        CMPEQC(R18,18,R0)
        FAIL_F(R0,35)

test36:
        SRA(R12,R2,R19) XNOR(R16,R19,R19) XNORC(R19,0,R19)
        CMPEQC(R19,19,R0)
        FAIL_F(R0,36)

test37:
        ADDC(R31,31,R20) ANDC(R20,20,R20)
        CMPEQC(R20,20,R0)
        FAIL_F(R0,37)

test38:
        ORC(R20,1,R21)
        CMPEQC(R21,21,R0)
        FAIL_F(R0,38)

test39:
        ADDC(R17,5,R22)
        CMPEQC(R22,22,R0)
        FAIL_F(R0,39)

test40:
        XOR(R22,R1,R23)
        CMPEQC(R23,23,R0)
        FAIL_F(R0,40)

test41:
        ANDC(R23,0xFC,R24) ADD(R24,R4,R24)
        CMPEQC(R24,24,R0)
        FAIL_F(R0,41)

test42:
        ADD(R23,R24,R25) ADDC(R25,-22,R25)
        CMPEQC(R25,25,R0)
        FAIL_F(R0,42)

test43:
        SHL(R3,R3,R26) OR(R2,R26,R26)
        CMPEQC(R26,26,R0)
        FAIL_F(R0,43)

test44:
        ADD(R14,R13,R27)
        CMPEQC(R27,27,R0)
        FAIL_F(R0,44)

test45:
        SUBC(R23,-5,R28)
        CMPEQC(R28,28,R0)
        FAIL_F(R0,45)

test46:
        SUBC(R31,-29,R29)
        CMPEQC(R29,29,R0)
        FAIL_F(R0,46)

test47:
        ADDC(R31,31,R30) SHRC(R30,1,R30) SHLC(R30,1,R30)
        CMPEQC(R30,30,R0)
        FAIL_F(R0,47)
        BF(R0,Error,R0)

test48:
        ADDC(30,17,R31)
    FAIL_T(R31,48)

// check out JMPs

test50:
        CMOVE(jmp2,R17)
        SHLC(R1,31,R1)
        OR(R1,R17,R17)
        JMP(R17,LP)
jmp1 = .
        FAIL_F(R31,50)          // shouldn't execute this!
jmp2 = .
test51:
        SHLC(LP,1,LP)           // get rid of kernel-mode bit
        SHRC(LP,1,LP)
        CMPEQC(LP,jmp1,R23)     // see if LP was filled correctly
        FAIL_F(R23,51)


// check out LDR/ADD/SUB
test52:
        LDR(CA,R24)
        LDR(C5,R25)
        ADD(R24,R25,R26)        // 0xAAAAAAAA + 0x55555555 = -1
        CMPEQC(R26,-1,R27)
        FAIL_F(R27,52)
test53:
        ADD(R24,R24,R26)        // 0xAAAAAAAA + 0xAAAAAAAA = 0x55555554
        SUB(R26,R25,R26)
        CMPEQC(R26,-1,R27)
        FAIL_F(R27,53)
test54:
        ADD(R25,R25,R26)        // 0x55555555 + 0x55555555 = 0xAAAAAAAA
        SUB(R26,R24,R26)
        FAIL_T(R26,54)
test55:
        CMOVE(1,R17)
        SUBC(R17,1,R18)         // test carry propagation
        FAIL_T(R18,55)


// test boolean operations
test56:
        CMOVE(0x0F0F,R11)
        CMOVE(0x7F00,R12)
        AND(R11,R12,R13)        // 0x0F0F & 0x7F00 = 0x0F00
        CMPEQC(R13,0x0F00,R14)
        FAIL_F(R14,56)
test57:
        OR(R11,R12,R13)         // 0x0F0F // 0x7F00 = 0x7F0F
        CMPEQC(R13,0x7F0F,R14)
        FAIL_F(R14,57)
test58:
        XOR(R11,R12,R13)        // 0x0F0F ^ 0x7F00 = 0x700F
        CMPEQC(R13,0x700F,R14)
        FAIL_F(R14,58)
test59:
        XNOR(R11,R12,R13)       // ~(0x0F0F ^ 0x7F00) = 0xFFFF8FF0
        CMPEQC(R13,0x8FF0,R14)
        FAIL_F(R14,59)


// test shifts
test60:
        CMOVE(1,R27)
        SHLC(R27,32,R28)        // should do nothing
        CMPEQC(R28,1,R29)
        FAIL_F(R29,60)
test61:
        SHLC(R27,31,R28)        // 1 << 31 = 0x80000000
        SRAC(R28,17,R26)        // 0x80000000 >>(signed) 17 = 0xFFFFC000
        CMPEQC(R26,0xC000,R25)
        FAIL_F(R25,61)
test62:
        SHRC(R28,17,R26)        // 0x80000000 >>(unsigned) 17 = 0x00004000
        CMPEQC(R26,0x4000,R25)
        FAIL_F(R25,62)


// test illegal operations
test63:
        CMOVE(0,XP)
        CMOVE(0,R1)
        LONG((0x00 << 26)+20)      // illegal operation // for circuit

        SHLC(XP,1,XP)           // get rid of kernel-mode bit
        SHRC(XP,1,XP)
        CMPEQC(XP,.-8,R0)
        FAIL_F(R0,63)
test64:                                 // test remaining illegal operations
        LONG(0x01 << 26)
        LONG(0x02 << 26)
        LONG(0x03 << 26)
        LONG(0x04 << 26)
        LONG(0x05 << 26)
        LONG(0x06 << 26)
        LONG(0x07 << 26)
        LONG(0x08 << 26)
        LONG(0x09 << 26)
        LONG(0x0A << 26)
        LONG(0x0B << 26)
        LONG(0x0C << 26)
        LONG(0x0D << 26)
        LONG(0x0E << 26)
        LONG(0x0F << 26)
        LONG(0x10 << 26)
        LONG(0x11 << 26)
        LONG(0x12 << 26)
        LONG(0x13 << 26)
        LONG(0x14 << 26)
        LONG(0x15 << 26)
        LONG(0x16 << 26)
        LONG(0x17 << 26)
        LONG(0x1A << 26)
        LONG(0x1E << 26)
        LONG(0x27 << 26)
        LONG(0x2F << 26)
        LONG(0x37 << 26)
        LONG(0x3F << 26)
        CMPEQC(R1,30,R0)
        FAIL_F(R0,64)

// test load and store
test65:
        CMOVE(LDST,R4)          // load base reg
        LDR(CA,R9)
        XORC(R9,-1,R0)          // R0 <- 0x55555555
        ST(R9,0,R4)             // try some stores
        ST(R0,LDST+4,R31)       // check out bypassing (if any!)
        ST(R9,8,R4)
        ST(R0,LDST+12,R31)
        LD(R4,0,R6)
        LDR(C5,R5)
        CMPEQ(R6,R9,R0)         // check 1-stage stall
        FAIL_F(R0,65)
test66:
        LD(R31,LDST+4,R7)
        CMPEQ(R7,R5,R0)         // check 2-stage stall
        FAIL_F(R0,66)
test67:
        LD(R4,8,R8)
        CMPEQ(R8,R9,R0)
        FAIL_F(R0,67)
test68:
        LD(R31,LDST+12,R9)
        CMPEQ(R9,R5,R0)
        FAIL_F(R0,68)

// finally, check out some bypass paths (just in case...)
test69:
        ADD(R31,R20,R0)         // R20 = 20
        ADD(R0,R0,R1)           // bypass from ALU stage
        ADD(R0,R0,R2)           // bypass from MEM stage
        ADD(R0,R0,R3)           // bypass from WB stage
        ADD(R0,R0,R4)           // read from register
        ADD(R1,R2,R5)
        ADD(R3,R5,R5)
        ADD(R4,R5,R5)
        CMPEQC(R5,160,R0)
        FAIL_F(R0,69)
test70:
        BNE(R31,.+4,R0)         // R0 = bypass
bypass: ADD(R0,R0,R1)   // bypass PC from ALU stage
        ADD(R0,R0,R2)           // bypass PC from MEM stage
        ADD(R0,R0,R3)           // bypass PC from WB stage
        ADD(R0,R0,R4)           // read from register
        ADD(R1,R2,R5)
        ADD(R3,R5,R5)
        ADD(R4,R5,R5)
        CMPEQC(R5,8*bypass,R0)
        FAIL_F(R0,70)

///////////////////////////////
// check interrupts  [Can't run this test in BSIM!]
/*
test71:
        CMOVE(.+8,R0)
// this won't work in BSIM
        JMP(R0)                 // leave supervisor mode

//// FOR CIRCUIT ONLY
        BR(ifail)               // this should be interrupted
ixp:
        BR(iokay)               // this should be executed on return
ifail:
        FAIL_F(R31,71)          // report error
iokay:
test72:
        CMPEQC(XP,ixp,R0)       // see if XP has correct value
        FAIL_F(R0,72)
/// end of for CIRCUIT
*/
///////////////////////////////
/// FOR BSIM only

// in BSIM, use this in place of test71
        CMOVE(xxx,R0)
        JMP(R0)                 // leave supervisor mode
        ADDC(R31,R31,R31)
        ADDC(R31,R31,R31)
xxx:    ADDC(R31,R31,R31)    // interrupt
        ADDC(R31,R31,R31)    // JMP(XP) interrupt return
        ADDC(R31,R31,R31)    // BR(iokay)
        ADDC(R31,R31,R31)    // CMPEQC
        ADDC(R31,R31,R31)    // branch in FAIL_F

/// end of for BSIM
///////////////////////////////
        
// make sure we can't jump from user mode to supervisor mode
test73:
        CMOVE(JTtest,R0)
        CMOVE(1,R1)
        SHLC(R1,31,R1)
        OR(R1,R0,R0)    // turn on supervisor bit in jump target
        JMP(R0,R1)                      // jumps to next instruction
JTtest:
        BR(.+4,R0)              // see what current PC is
        CMPEQC(R0,JTtest+4,R1)  // shouldn't have supervisor bit on
        FAIL_F(R1,73)
        
// test Beta's Z logic
test74:
        LD(CA,r0)
        BEQ(r0,test74_fail)
        LD(C5,r0)
        BEQ(r0,test74_fail)
        CMOVE(0,r0)
        BNE(r0,test74_fail)
        CMOVE(1,r1)
        CMOVE(32,r2)
t74_loop:
        BNE(r1,test74_okay)
test74_fail:
        CMOVE(74,r0)
        BR(Error)
test74_okay:
        SHLC(r1,1,r1)
        SUBC(r2,1,r2)
        BNE(r2,t74_loop)

// all done!

Done:   ADDC(R31,0,R31)
//        BR(Done)       // actual hardware test loops
        HALT()         // bsim version halts

CA:     LONG(0xAAAAAAAA)
C5:     LONG(0x55555555)
LDST:   LONG(0)
        LONG(0)
        LONG(0)
        LONG(0)

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
            var cinfo = $('.cache-information');
            $(window).on('resize cache-resize',function() {
                var height = window_height() - difference;
                if (cinfo.is(':visible')) height -= cinfo.outerHeight();
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

    function setup_tab(name,contents,select,read_only) {
        // if initial contents looks like a URL, load it!
        var load = contents.lastIndexOf('url:',0) === 0;
        var doc = editor.openTab(name,load ? 'Loading '+contents : contents,select,null,read_only);
        if (load) {
            $.ajax(contents.substr(4),{
                dataType: 'text',
                error: function(jqXHR,textStatus,errorThrown) {
                    editor.load_initial_contents(doc,'Oops, error loading '+contents);
                },
                success: function(data,jqXHR,textStatus,errorThrown) {
                    editor.load_initial_contents(doc,data);
                }
            });
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

        // use text from iframe body, if any
        var text = $('#initial-state').html();
        if (text) {
            // start with state specified by iframe body
            var s = JSON.parse(text);
            // extend with state received from server
            $.extend(s,JSON.parse(stateStr));
            // make result the updated state
            stateStr = JSON.stringify(s);
        }

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
                   setup_tab(name,contents,false,true);
                   //editor.openTab(name,contents,false,null,true);
               });
        $.each(configuration.state || {},
               function (name,contents) {
                   setup_tab(name,contents,first,false);
                   //editor.openTab(name,contents,first);
                   first = false;
               });

        $('.editor-file-control').hide();     // hide file buttons
        $('#editor .nav-tabs .close').hide();  // hide close button on tab(s)
    };
    
    // Establish a channel only if this application is embedded in an iframe.
    // This will let the parent window communicate with this application using
    // RPC and bypass SOP restrictions.
    var channel;
    if (window.parent !== window && channel === undefined) {
        channel = Channel.build({
            window: window.parent,
            origin: "*",
            scope: "JSInput"
        });

        channel.bind("getGrade", BSim.getGrade);
        channel.bind("getState", BSim.getState);
        channel.bind("setState", BSim.setState);

        // make iframe resizable if we can.  This may fail if we don't have
        // access to our parent...
        try {
            // look through all our parent's iframes
            $('iframe',window.parent.document).each(function () {
                // is this iframe us?
                if (this.contentWindow == window) {
                    // yes! so add css to enable resizing
                    $(this).css({resize:'both', overflow:'auto'});
                }
            });
        } catch (e) {
        }
    }

});
