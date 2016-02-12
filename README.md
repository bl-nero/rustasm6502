# A 6502 assembler as a Rust macro [![Build Status](https://travis-ci.org/jonas-schievink/rustasm6502.svg?branch=master)](https://travis-ci.org/jonas-schievink/rustasm6502)

**If you want to lose a bit of your sanity, then this crate is for you!**

(no but seriously, you want to use a *macro* to assemble stuff?)

[Documentation](http://jonas-schievink.github.io/rustasm6502/rustasm6502/)

## Implemented Features

* Full compile-time assembly to executable machine code
* Label support (`loop: jmp loop`, `beq start`)
* Supports all official 6502 opcodes and addressing modes
