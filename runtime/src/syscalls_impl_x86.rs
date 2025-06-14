use core::arch::asm;
use libtock_platform::{syscall_class, RawSyscalls, Register};

unsafe impl RawSyscalls for crate::TockSyscalls {
    // Yield 1 is used for yield_wait
    unsafe fn yield1([Register(r0)]: [Register; 1]) {
        unsafe {
            asm!(
                // Save the address following the `int` instruction and push it
                // on the stack.
                "leal 2f, %eax",
                "push %eax",
                "movl $0, %eax",
                "int $0x40",
                "2:",

                inlateout("ebx") r0 => _,

                // The following registers are clobbered by the syscall
                out("eax") _,
                out("ecx") _,
                out("edx") _,
                out("edi") _,
                options(att_syntax),
            );
        }
    }

    // Yield 2 is used for yield_no_wait
    unsafe fn yield2([Register(r0), Register(r1)]: [Register; 2]) {
        unsafe {
            asm!(
                // Save the address following the `int` instruction and push it
                // on the stack.
                "leal 2f, %eax",
                "push %eax",
                "movl $0, %eax",
                "int $0x40",
                "2:",

                inlateout("ebx") r0 => _,
                inlateout("ecx") r1 => _,

                // The following registers are clobbered by the syscall
                out("eax") _,
                out("edx") _,
                out("edi") _,
                options(att_syntax)
            );
        }
    }

    unsafe fn syscall1<const CLASS: usize>([Register(mut r0)]: [Register; 1]) -> [Register; 2] {
        // This is memop, the only syscall class that syscall1 supports
        let r1;
        unsafe {
            asm!(
                "movl  $5, %eax",
                "int $0x40",

                inlateout("ebx") r0,
                out("ecx") r1,

                // The following registers are clobbered by the syscall
                out("eax") _,
                out("edx") _,
                out("edi") _,
                options(att_syntax),
            );
        }
        [Register(r0), Register(r1)]
    }

    unsafe fn syscall2<const CLASS: usize>(
        [Register(mut r0), Register(mut r1)]: [Register; 2],
    ) -> [Register; 2] {
        let cmd: u32 = match CLASS {
            syscall_class::MEMOP => 5,
            syscall_class::EXIT => 6,
            _ => unreachable!(),
        };

        unsafe {
            asm!(
                "int $0x40",

                inlateout("ebx") r0,
                inlateout("ecx") r1,
                inlateout("eax") cmd => _,

                // The following registers are clobbered by the syscall
                out("edx") _,
                out("edi") _,
                options(att_syntax),
            );
        }

        [Register(r0), Register(r1)]
    }

    unsafe fn syscall4<const CLASS: usize>(
        [Register(mut r0), Register(mut r1), Register(mut r2), Register(mut r3)]: [Register; 4],
    ) -> [Register; 4] {
        let cmd: u32 = match CLASS {
            syscall_class::SUBSCRIBE => 1,
            syscall_class::COMMAND => 2,
            syscall_class::ALLOW_RW => 3,
            syscall_class::ALLOW_RO => 4,
            _ => unreachable!(),
        };
        unsafe {
            asm!(
                "int $0x40",
                
                inlateout("ebx") r0,
                inlateout("ecx") r1,
                inlateout("edx") r2,
                inlateout("edi") r3,

                inlateout("eax") cmd => _,

                options(att_syntax),
            );
        }

        [Register(r0), Register(r1), Register(r2), Register(r3)]
    }
}
