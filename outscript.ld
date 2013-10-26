
/* The linker command script */

/* Set the entry point */
/* ENTRY    (opec_cusBinHeader)   */
ENTRY    (mos_BinHeader)   


/* Set the memory mapping */
MEMORY 
{
   ROM_MAP (rx)   : ORIGIN = 0x00220000, LENGTH = 8192k 
/* 1MB+ Memory */
/*   RAM_MAP (!rx)  : ORIGIN = 0x18100000, LENGTH = 8192k  */
/*   RAM_MAP (!rx)  : ORIGIN = 0x18100000, LENGTH = 16384k */
/* 256K Memory */
   RAM_MAP (!rx)  : ORIGIN = 0x180C0000, LENGTH = 8192k  

}
/*
PHDRS
{
   appli PT_LOAD ;
}
*/
SECTIONS
{
   .oat_header :
   {
      mos_header.o ( .rodata* )
   } > ROM_MAP /*:appli*/
   
   .text : 
   {
      /* RO and ER_RO symbols are provided for legacy OS support */
      /* define the value of the begin of code section */
      PROVIDE(Image$$APP_RO$$Base = .);
      PROVIDE(Image$$RO$$Base = .);
      *( .text ) 
      *( .rodata* )
      *(.glue_7t) *(.glue_7)
	  *(.ctors) *(.dtors)
	  *(.jcr)
	  *(.init) *(.fini)
      /* define the value of the end of code section */
      PROVIDE(Image$$APP_RO$$Limit = .);
      PROVIDE(Image$$RO$$Limit = .);
      /* define the length of the code section */
      PROVIDE(Image$$APP_RO$$Length = SIZEOF(.text));
      PROVIDE(Image$$ER_RO$$Length = SIZEOF(.text));
      _etext = .;
   } > ROM_MAP /*:appli*/

  .data : AT (_etext) 
/*  .data : */
   {
      /* RW and ER_RW symbols are provided for legacy OS support */
      /* define the value of the begin of data initialized section */
      PROVIDE(Image$$APP_DATA$$Base = .);
      PROVIDE(Image$$RW$$Base = .);
      *( .data ) 
      /* define the value of the end of data initialized section */
      PROVIDE(Image$$APP_DATA$$Limit = .);
      PROVIDE(Image$$RW$$Limit = .);
      /* define the length of the data initialized section */
      PROVIDE(Image$$APP_DATA$$Length = SIZEOF(.data));
      PROVIDE(Image$$ER_RW$$Length = SIZEOF(.data));
      _edata = .;
   } > RAM_MAP /*:appli*/

   .bss : AT (_edata) 
/*   .bss :  */
   { 
	   __bss_start__ = .;
      /* ZI symbol is provided for legacy OS support */
      /* define the value of the begin of data zero initialized section */
      PROVIDE(Image$$APP_BSS$$Base = .);
      PROVIDE(Image$$ZI$$Base = .);
      *( .bss ) 
      *( COMMON ) 
      /* define the value of the end of data zero initialized section */
      PROVIDE(Image$$APP_BSS$$Limit = .);
      PROVIDE(Image$$ZI$$Limit = .);
      /* define the length of the data zero initialized section */
      PROVIDE(Image$$APP_BSS$$ZI$$Length = SIZEOF(.bss));
      PROVIDE(Image$$ER_ZI$$ZI$$Length = SIZEOF(.bss));
      _ebss = .;
	   __bss_end__ = .;
   } > RAM_MAP /*:appli*/
   
   .uninit (NOLOAD) :
   {
      /* define the value of the begin of uninitialized section */
      PROVIDE(Image$$APP_UNINIT$$Base = .); 
      * (UNINIT)
      /* define the value of the end of uninitialized section */
      PROVIDE(Image$$APP_UNINIT$$Limit = .);
      PROVIDE(end = .);
      /* define the length of the uninitialized section */
      PROVIDE(Image$$APP_UNINIT$$ZI$$Length = SIZEOF(.uninit));
   } > RAM_MAP /*:appli*/
   
   . = ALIGN(4);
}

