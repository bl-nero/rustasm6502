//! A MOS6502 assembler implemented as a macro.
//!
//! **Note**: Due to limitations in Rust macros, syntax for absolute addressing modes requires the
//! use of `abs` after the mnemonic (eg. `lda abs 0xffff`).
//!
//! # Example
//!
//! ```
//! #[macro_use] #[no_link]
//! extern crate rustasm6502;
//!
//! fn main() {
//!     let machine_code = assemble6502! {
//!         lda #0x00
//!         ldx #0x00
//!         ldy #0x00
//!         txs
//!
//!     main:
//!         adc 0xff        // Zero-Page
//!         adc abs 0x1234  // Absolute address 0x1234
//!         beq main
//!     end:
//!         jmp end
//!     };
//!
//!     assert_eq!(machine_code, [
//!         0xA9, 0x00,
//!         0xA2, 0x00,
//!         0xA0, 0x00,
//!         0x9A,
//!
//!         0x65, 0xff,
//!         0x6D, 0x34, 0x12,   // Little-endian
//!         0xF0, (-7i8) as u8,
//!
//!         0x4C, 14, 0,
//!     ]);
//! }
//! ```

/// A compile-time map from identifiers to arbitrary (heterogeneous) expressions
#[macro_export]
#[doc(hidden)]
macro_rules! ident_map {
    ( $name:ident = { $($key:ident => $e:expr),* $(,)* } ) => {
        macro_rules! $name {
            $(
                ( $key ) => { $e };
            )*
            // Empty invocation expands to nothing. Needed when the map is empty.
            () => {};
        }
    };
}

/// Given an array of machine code (Bytes), returns its length.
/// FIXME Don't go through an array
#[macro_export]
#[doc(hidden)]
macro_rules! codelen {
    () => { 0 };
    ( $($c:expr),+ ) => { [$($c),+].len() };
}

/// Replace elements of "arrays" of expressions with sorted replacements by
/// seeking along a "positional array" containing n expressions, then replacing
/// elements in the source array.
///
/// Expands to the first array with replacements applied. The length doesn't
/// change.
#[macro_export]
#[doc(hidden)]
macro_rules! lockstep_replace {
    ( [ $($result:expr),* ], [], ) => {
        // `src` is empty, no relocations. We're done!
        [ $($result),* ]
    };
    ( [ $($result:expr),* ], [ $($src:expr,)+ ], ) => {
        // Empty list of replacements, but still `src` to go
        [ $($result,)* $($src),+ ]
    };
    ( [ $($result:expr),* ], [ $($src:expr,)* ], [], [], $( [ $($pos:expr,)* ], [ $($rep:expr,)* ], )* ) => {
        // All replacements applied. Pop the current replacement and continue.
        lockstep_replace!(
            [ $($result),* ],
            [ $($src,)* ],
            $(
                [ $($pos,)* ],
                [ $($rep,)* ],
            )*
        )
    };
    ( [ $($result:expr),* ], [ $src1_replaced:expr, $($src:expr,)* ], [], [ $rep1:expr, $($rep_rest:expr,)* ], $( [ $pos1:expr, $($pos:expr,)* ], [ $($rep:expr,)* ], )* ) => {
        // Position of a replacement reached (or: inside a replacement)
        // Coupled with a seek step
        lockstep_replace!(
            [ $($result,)* $rep1 ],
            [ $($src,)* ],
            [],
            [ $($rep_rest,)* ],
            $(
                [ $($pos,)* ],
                [ $($rep,)* ],
            )*
        )
    };
    ( [ $($result:expr),* ], [ $src1:expr, $($src:expr,)* ], $( [ $pos1:expr, $($pos:expr,)* ], [ $($rep:expr,)* ], )+ ) => {{
        // Seek to next replacement position (simultaneously for all
        // replacements)
        lockstep_replace!(
            [ $($result,)* $src1 ],
            [ $($src,)* ],
            $(
                [ $($pos,)* ],
                [ $($rep,)* ],
            )+
        )
    }};
}

/// Performs relocation of machine code based on given labels and relocations.
/// Looks up label names in an `ident_map`. Expands to the relocated machine
/// code.
///
/// Relocation formats:
/// { $label as ABS16 @ [$lockstepmcpos] }
#[macro_export]
#[doc(hidden)]
macro_rules! reloc {
    ( [ $( [ $($pos:expr),* ], [ $($rep:expr),* ] ),* ], $lblmap:ident, [ $($mcode:expr),* ], [/* empty relocation list */] ) => {{
        lockstep_replace!([], [ $($mcode,)* ], $( [ $($pos,)* ], [ $($rep,)* ], )*)
    }};
    ( [ $( [ $($pos:expr),* ], [ $($rep:expr),* ] ),* ], $lblmap:ident, [ $($mcode:expr),* ], [ { $lbl:ident as ABS16 @ [$($lockstepmcpos:expr),*] } $(,$reloc:tt)* ] ) => {
        // Replace 2 Bytes with the absolute address
        // Relocation position is given as "lock-step MC pos", an expression
        // list that's as long as all mcode before the relocation should happen.
        reloc!(
            [ $( [ $($pos),* ], [ $($rep),* ] ,)*
            [ $($lockstepmcpos),* ], [ ( $lblmap!($lbl) as u8 ), ($lblmap!($lbl) >> 8) as u8 ] ],
            $lblmap, [ $($mcode),* ], [ $($reloc),* ])
    };
    ( [ $( [ $($pos:expr),* ], [ $($rep:expr),* ] ),* ], $lblmap:ident, [ $($mcode:expr),* ], [ { $lbl:ident as PCREL @ [$($lockstepmcpos:expr),*] } $(,$reloc:tt)* ] ) => {
        // Replace 1 Byte with the PC relative address
        // PC is the program counter *after* the relocated offset (the length of the
        // `$lockstepmcpos` array + 1), so we need to subtract 1 additional byte.
        reloc!(
            [ $( [ $($pos),* ], [ $($rep),* ] ,)*
            [ $($lockstepmcpos),* ], [ ( $lblmap!($lbl) as i32 - codelen!($($lockstepmcpos),*) as i32 - 1 ) as u8 ] ],
            $lblmap, [ $($mcode),* ], [ $($reloc),* ])
    };
}

#[macro_export]
#[doc(hidden)]
macro_rules! asm_ {
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        // EOF
    ) => {{
        ident_map!(labelmap = {
            $($lbl => $lblval),*
        });
        reloc!([], labelmap, [ $($mcode),* ], [ $($reloc),* ])
    }};

    // ==================================================================================
    // ==================================================================================
    // ==================================================================================

    // Opcode assembly table.
    // Modeled after http://obelisk.me.uk/6502/reference.html
    // Note that the weird order is required because macros try to match each arm in order, but
    // don't backtrack when a NT is parsed

    // ADC
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        adc # $imm:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x69, $imm ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        adc ($ind:tt, x)
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x61, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        adc ($ind:tt), y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x71, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        adc $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x75, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        adc abs $abs:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x7D, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        adc abs $abs:tt, y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x79, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        adc abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x6D, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        adc $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x65, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // AND
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        and # $imm:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x29, $imm ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        and ($ind:tt, x)
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x21, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        and ($ind:tt), y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x31, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        and $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x35, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        and abs $abs:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x3D, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        and abs $abs:tt, y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x39, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        and abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x2D, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        and $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x25, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // ASL
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        asl $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x16, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        asl abs $abs:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x1E, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        asl abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x0E, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        asl $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x06, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        asl     // Accumulator
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x0A ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // BCC
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        bcc $label:ident
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x90, 0x00 ],
            [ $($lbl => $lblval),* ], [ $($reloc,)* { $label as PCREL @ [$($mcode,)* 0x90] } ], $($rest)*)
    };
    // BCS
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        bcs $label:ident
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xB0, 0x00 ],
            [ $($lbl => $lblval),* ], [ $($reloc,)* { $label as PCREL @ [$($mcode,)* 0xB0] } ], $($rest)*)
    };
    // BEQ
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        beq $label:ident
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xF0, 0x00 ],
            [ $($lbl => $lblval),* ], [ $($reloc,)* { $label as PCREL @ [$($mcode,)* 0xF0] } ], $($rest)*)
    };

    // BIT
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        bit abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x2C, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        bit $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x24, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // BMI
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        bmi $label:ident
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x30, 0x00 ],
            [ $($lbl => $lblval),* ], [ $($reloc,)* { $label as PCREL @ [$($mcode,)* 0x30] } ], $($rest)*)
    };
    // BNE
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        bne $label:ident
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xD0, 0x00 ],
            [ $($lbl => $lblval),* ], [ $($reloc,)* { $label as PCREL @ [$($mcode,)* 0xD0] } ], $($rest)*)
    };
    // BPL
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        bpl $label:ident
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x10, 0x00 ],
            [ $($lbl => $lblval),* ], [ $($reloc,)* { $label as PCREL @ [$($mcode,)* 0x10] } ], $($rest)*)
    };

    // BRK
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        brk
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x00 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // BVC - Branch if Overflow flag clear
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        bvc $label:ident
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x50, 0x00 ],
            [ $($lbl => $lblval),* ], [ $($reloc,)* { $label as PCREL @ [$($mcode,)* 0x50] } ], $($rest)*)
    };
    // BVS - Branch if Overflow flag set
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        bvs $label:ident
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x70, 0x00 ],
            [ $($lbl => $lblval),* ], [ $($reloc,)* { $label as PCREL @ [$($mcode,)* 0x70] } ], $($rest)*)
    };

    // CLC - Clear Carry flag
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        clc
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x18 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // CLD - Clear decimal mode flag
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cld
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xD8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // CLI - Clear interrupt disable flag
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cli
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x58 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // CLV - Clear overflow flag
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        clv
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xB8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // CMP
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cmp # $imm:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xC9, $imm ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cmp ($ind:tt, x)
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xC1, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cmp ($ind:tt), y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xD1, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cmp $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xD5, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cmp abs $abs:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xDD, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cmp abs $abs:tt, y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xD9, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cmp abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xCD, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cmp $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xC5, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // CPX
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cpx # $imm:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xE0, $imm ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cpx abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xEC, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cpx $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xE4, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // CPY
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cpy # $imm:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xC0, $imm ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cpy abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xCC, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        cpy $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xC4, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // DEC - Decrement memory
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        dec $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xD6, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        dec abs $abs:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xDE, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        dec abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xCE, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        dec $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xC6, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // DEX - Decrement X register
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        dex
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xCA ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // DEY - Decrement Y register
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        dey
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x88 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // EOR
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        eor # $imm:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x49, $imm ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        eor ($ind:tt, x)
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x41, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        eor ($ind:tt), y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x51, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        eor $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x55, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        eor abs $abs:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x5D, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        eor abs $abs:tt, y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x59, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        eor abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x4D, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        eor $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x45, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // INC - Increment memory
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        inc $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xF6, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        inc abs $abs:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xFE, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        inc abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xEE, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        inc $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xE6, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // INX
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        inx
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xE8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // INY - Increment Y register
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        iny
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xC8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // JMP
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        jmp $label:ident    // jmp abs
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x4C, 0x00, 0x00 ],
            [ $($lbl => $lblval),* ], [ $($reloc,)* { $label as ABS16 @ [$($mcode,)* 0x4C] } ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        jmp ($ind:tt)
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x6C, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // JSR
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        jsr $label:ident
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x20, 0x00, 0x00 ],
            [ $($lbl => $lblval),* ], [ $($reloc,)* { $label as ABS16 @ [$($mcode,)* 0x4C] } ], $($rest)*)
    };

    // LDA
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        lda # $imm:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xA9, $imm ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        lda ($ind:tt, x)
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xA1, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        lda ($ind:tt), y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xB1, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        lda $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xB5, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        lda abs $abs:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xBD, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        lda abs $abs:tt, y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xB9, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        lda abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xAD, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        lda $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xA5, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // LDX
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ldx # $imm:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xA2, $imm ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ldx $zp:tt, y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xB6, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ldx abs $abs:tt, y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xBE, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ldx abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xAE, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ldx $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xA6, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // LDY
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ldy # $imm:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xA0, $imm ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ldy $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xB4, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ldy abs $abs:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xBC, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ldy abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xAC, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ldy $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xA4, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // LSR - Logical Shift Right
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        lsr $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x56, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        lsr abs $abs:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x5E, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        lsr abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x4E, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        lsr $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x46, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        lsr     // Accumulator
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x4A ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // NOP
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        nop
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xEA ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // ORA
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ora # $imm:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x09, $imm ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ora ($ind:tt, x)
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x01, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ora ($ind:tt), y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x11, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ora $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x15, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ora abs $abs:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x1D, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ora abs $abs:tt, y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x19, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ora abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x0D, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ora $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x05, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // PHA - Push accumulator
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        pha
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x48 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // PHP - Push processor status
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        php
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x08 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // PLA - Pull accumulator (Pop)
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        pla
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x68 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // PLP - Pull processor status
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        plp
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x28 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // ROL - Rotate Left
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        rol $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x36, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        rol abs $abs:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x3E, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        rol abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x2E, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        rol $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x26, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        rol     // Accumulator
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x2A ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // ROR - Rotate Right
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ror $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x76, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ror abs $abs:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x7E, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ror abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x6E, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ror $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x66, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        ror     // Accumulator
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x6A ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // RTI
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        rti
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x40 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // RTS
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        rts
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x60 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // SBC
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sbc # $imm:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xE9, $imm ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sbc ($ind:tt, x)
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xE1, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sbc ($ind:tt), y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xF1, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sbc $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xF5, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sbc abs $abs:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xFD, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sbc abs $abs:tt, y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xF9, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sbc abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xED, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sbc $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xE5, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // SEC - Set Carry Flag
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sec
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x38 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // SED - Set Decimal Flag
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sed
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xF8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // SEI - Set Interrupt Disable
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sei
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x78 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // STA
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sta ($ind:tt, x)
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x81, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sta ($ind:tt), y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x91, $ind ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sta $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x95, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sta abs $abs:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x9D, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sta abs $abs:tt, y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x99, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sta abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x8D, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sta $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x85, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // STX
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        stx $zp:tt, y
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x86, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        stx abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x8E, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        stx $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x86, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // STY
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sty $zp:tt, x
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x94, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sty abs $abs:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x8C, ($abs as u16) as u8, (($abs as u16) >> 8) as u8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        sty $zp:tt
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x84, $zp ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // TAX
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        tax
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xAA ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // TAY
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        tay
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xA8 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // TSX
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        tsx
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0xBA ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // TXA
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        txa
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x8A ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // TXS
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        txs
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x9A ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };
    // TYA
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        tya
    $($rest:tt)* ) => {
        asm_!([ $($mcode,)* 0x98 ], [ $($lbl => $lblval),* ], [ $($reloc),* ], $($rest)*)
    };

    // ==================================================================================
    // ==================================================================================
    // ==================================================================================

    // Check for labels
    ( [ $($mcode:expr),* ], [ $($lbl:ident => $lblval:expr),* ], [ $($reloc:tt),* ],
        $label:ident :
    $($rest:tt)* ) => {{
        asm_!(
            [ $($mcode),* ],
            [ $($lbl => $lblval,)* $label => codelen!($($mcode),*) ],
            [ $($reloc),* ],
            $($rest)*
        )
    }};
}

/// Entry point for the macro-based MOS6502 assembler.
///
/// Expands to a fixed-size `u8` array containing the assembled machine code.
///
/// **Note**: Any errors in the assembly will be reported as inscrutable expansion errors. This is
/// a limitation in Rust's current macro implementation.
#[macro_export]
macro_rules! assemble6502 {
    ( $($tokens:tt)* ) => {
        asm_!([], [], [], $($tokens)*)
    };
}

#[test]
fn test_ident_map() {
    ident_map!(my_map = {
        main => 0,
        end => 0x45,
    });
    assert_eq!(my_map!(main), 0);
    assert_eq!(my_map!(end), 0x45);
    // Unknown keys cause syntax errors (can't test that, but believe me :P)
}

/// Test simple label relocation
#[test]
fn test_simple_jmp() {
    let mcode = assemble6502!(
        start: jmp start
    );
    assert_eq!(mcode, [ 0x4C, 0x00, 0x00 ]);
}

/// Has to work without any relocations (label references)
#[test]
fn test_no_reloc() {
    let mcode = assemble6502!(
        start:
            lda #0xfb
    );
    assert_eq!(mcode, [ 0xA9, 0xFB ]);
}

/// Has to work without any labels
#[test]
fn test_no_label() {
    let mcode = assemble6502!(
        lda #0xfb
        lda #0xab
    );
    assert_eq!(mcode, [ 0xA9, 0xFB, 0xA9, 0xAB ]);
}

/// Tests multiple labels and relocated jumps, `lbl1` is unused
#[test]
fn test_labels() {
    let mcode = assemble6502!(
        start:
            lda #0x0f
        lbl1:
            lda #0x0f
            jmp main
        main:
            jmp start
    );
    assert_eq!(mcode, [ 0xA9, 0x0F, 0xA9, 0x0F, 0x4C, 0x07, 0x00, 0x4C, 0x00, 0x00 ]);
}

/// Tests all modes of the ADC instruction
#[test]
fn test_adc() {
    let mcode = assemble6502!(
        adc #0          // Immediate
        adc 0x12, x     // Zero Page indexed with X
        adc 0x34        // Zero Page
        adc abs 0x1234  // Absolute (0x1234)
        adc abs 0x1020, x   // Abs. indexed with X
        adc abs 0x3040, y   // Abs. indexed with Y
        adc (0x10, x)   // (Indirect, X)
        adc (0x01),y    // (Indirect),Y
    );
    assert_eq!(mcode, [
        0x69, 0x00,
        0x75, 0x12,
        0x65, 0x34,
        0x6D, 0x34, 0x12,
        0x7D, 0x20, 0x10,
        0x79, 0x40, 0x30,
        0x61, 0x10,
        0x71, 0x01
    ]);
}

/// Tests the pc-relative relocation.
#[test]
fn test_pcrel() {
    let mcode = assemble6502!(
        start:
            bcc start
            bcc main
        main:
            bcc start
    );
    assert_eq!(mcode, [
        0x90, 0xfe,
        0x90, 0x00,
        0x90, (-6i8) as u8,
    ]);
}
