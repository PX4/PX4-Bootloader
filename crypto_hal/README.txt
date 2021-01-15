Signing px4 firmware files and checking the signatures in bootloader

1) Bootloader with signature checking support

If the environment variable CRYPTO_HAL is defined while building the bootloader,
the bootloader will automatically include signature check support.

If the envirnoment variable CRYPTO_HAL is not defined, there is no extra code
or structures added to the bootloader.

By default the bootloader supports monocypher ed25519 signature check. In order
to add other signature/encryption methods, one must create another crypto-hal
implementation following the example in crypto_hal/monocypher

2) Using monocypher crypto-hal

To build the default signature checking support into bootloader, define the
following environment variables:

export CRYPTO_HAL=monocypher
export PUBLIC_KEY0=${PWD}/crypto_hal/test_key/key0.pub

And build the bootloader for your board, e.g:
make px4fmuv5_bl

The key file in the above example points to px4 test key. This key should be
replaced with public key from a self generated key pair

4) Structure of the expected table of contents in the px4 firmware

The format of table of contents is

magic (4 bytes)
version (4 bytes)

n \sz  4    4     4   4      1             1             1             1      4
   ----------------------------------------------------------------------------
1  |   name start end target signature_idx signature_key ecryption_key flags1 reserved
2  |   name start end target signature_idx signature_key ecryption_key flags1 reserved
.  |
.  |
N  |   name start end target signature_idx signature_key ecryption_key flags1 reserved

end (4 bytes)

Description of the fields:

magic:          always 0x00434f54, in ascii string "TOC"
version:        crypto version, reserved for vendor specific use

name:           the entry name (not used by bl)
start:          start address of the block
end:            end address of the block
target:         the block shall be copied to this address after all crypto
                actions have been taken (signature check, decrypting)
signature_idx:  entry index within the TOC to find signature block for this
signature_key:  public key number, which is used for sig check
encryption key: private key number, which is used for decrypting the block
flags1:         flags telling what to do with the block ( see chapter 5 )
reserved:       reserved/vendor specific

end:            always 0x00444e45, in ascii string "END"

The version and N entries are defined by the board for which the px4 is being
built for. The format of the each entry is:
name<4> start<4*> end<4*> signature_idx<1> signature_key<1> ecryption_key<1> flags1<1> reserved<4>

*) entry is 20 bytes on 32-bit targets. On a 64 bit targets native 64-bit build
   it would be 32 bytes

Each entry defines a block of contents for the binary. The bootloader goes
through the TOC line-by-line and performs any tasks defined by "flags".

The TOC is always contained within the first block, which is always checked for
valid signature. The exact placement of the TOC is right after the boot delay
signature "BOOT_DELAY_ADDRESS".

5) The TOC parsing in image_toc.c

The bootloader will find the TOC and parse each entry. Depending on "flags" of
the entry, it will perform actions on each entry. The currently supported flags
are:

TOC_FLAG1_BOOT:
  - This flag marks a bootable image; the bootloader will jump to this
  image after all the crypto actions have been taken. If no separate
  vector table exists in the TOC, the application's vector table is used

TOC_FLAG1_VTORS:
  - This flag marks a separate vector table block. The bootloader will
  set the interrupt vectors to this block. This will override the vector
  table from the application.

TOC_FLAG1_CHECK_SIGNATURE:
  - Any block marked with this flag in the TOC will be signature checked.
  if the check fails (for any entry marked with the flag), booting is prohibited

TOC_FLAG1_DECRYPT:
  - The block is encrypted and has to be decrypted before use. This kind of
  block can be targeted for either bootloader or px4 firmware (the TOC is also
  accessible in px4 firmware) to pass any secrets.
  An example bootloader usage is for a chipset which can execute code directly
  from an encrypted flash. The bootloader can then set up the relevant crypto
  accelerators.
  Another example is to pass new (private) keys or IP protected code to
  either bootloader or px4 firmware
  Currently this flag is ignored by the bootloader, as there is no private key
  storage implemented.

TOC_FLAG1_RDCT:
  - This flag marks that the block is an R&D certificate, unlocking some
  features for an individual device. This flag is reserved for future use,
  it is not yet implemented in bootloader.

- The "flags" don't control the signature check for the first block;
  this one contains the TOC, and it must be tamper protected as well

- Booting any image will always happen only after the whole TOC has been
  processed.

6) Notes

- A normal bootloader which doesn't do signature checking will boot also signed
  px4 binaries
- A bootloader which does the signature checking will boot only properly signed
  px4 binaries
- The public key for signature checking will be embedded in the bootloader code.
  In order to prevent booting unauthorized binaries in the device, one must
  ensure that the bootloader or the public key cannot be tampered with
  or by-passed completely. For example in STM32 hardware this requires at least
  locking the chip to RDP level 2, on other HW targets mechanisms differ.
- On STM32 platforms, remember that RDP level 2 lockdown is irreversible.
  To test the signing, the locking down is not required.
- Remember that it is a responsibility of the user of this feature to ensure
  that the chain of trust is maintained at desired level. The crypto-hal
  implementation is just a tool, one must understand how to use it.
