// Lab 3 template file
.data

.global login_name
login_name:
    .asciz "rafi"

.global puid
puid:
    .asciz "0033664036"

.balign 4

.equ TEST_SORT,         0x1
.equ TEST_STRCONCAT,    0x2
.equ TEST_FIB,          0x4
.equ TEST_BSEARCH,      0x8
.equ TEST_ALL,          (TEST_SORT | TEST_STRCONCAT | TEST_FIB | TEST_BSEARCH)

// Replace the TEST_ALL with the tests you want to run
// OR expression also support like the TEST_ALL above
// i.e. replace the TEST_ALL below with TEST_SORT | TEST_STRCONCAT
//      will test on the sort and strconcat
.global test
test:
    .word TEST_ALL


// Align to 4bytes boundary as we have a string above
.balign 4

.text
.global asm_sort_int
asm_sort_int:
    /* void asm_sort_int(int32_t* arr, uint32_t n)
     * relies on `qsort` in C standard library to 
     * sort in ascending order
     **/

    /* Enter your code after this comment */

    li x12,4
    la x13,asm_cmp

    addi sp, sp, -32
    sw ra, 0(sp)

    call qsort

    lw ra,0(sp)
    addi sp,sp,32

    /* Enter your code above this comment */
    ret

.global asm_cmp
asm_cmp:
    // int asm_cmp(const void * a, const void * b)
    // Compare function to int32_t
    // used for qsort
    /* Enter your code after this comment */
    lw t0,0(x10)
    lw t1,0(x11)

    blt t0,t1,R_neg // if t0<t1

    li x10,1
    ret
    R_neg:
     li x10,-1
     ret    
    /* Enter your code above this comment */
    ret

.global asm_strconcat
asm_strconcat:
    /* char* asm_strconcat(char * str1, char * str2)
     * First assign memory space with `malloc`, 
     * concatenating `str1` and `str2`, 
     * and return the resulted string.
     **/
    /* Enter your code after this comment */
   addi sp,sp,-32
   sw ra,0(sp)
   sw a0,4(sp)
   sw a1,8(sp)


   lw a0,8(sp)
   call strlen
   sw a0,16(sp)


   lw a0,12(sp)
   lw t1,16(sp)
   add a0,a0,t1
   addi a0,a0,1
   sw a0,24(sp)
   call malloc
   sw a0,20(sp)

   lw a0,20(sp)
   lw a1,4(sp)
   lw a2,12(sp)
   call memcpy


   lw a0,20(sp)
   lw t1,12(sp)
   add a0,a0,t1
   lw a1,8(sp)
   lw a2,16(sp)
   call memcpy

   lw a0,20(sp)
   lw t1,24(sp)
   add a0,a0,t1
   addi a0,a0,-1
   li t2,0
   sb t2,0(a0)

   lw a0,20(sp)
   lw ra, 0(sp)
   addi sp,sp,32   
    /* Enter your code above this comment */
    ret

.global asm_fib
asm_fib:
    /* uint32_t asm_fib(uint32_t n)
     * accept an index term `n` and return 
     * the Fn fibonacci term (F0 = 0, F1 = 1)
     **/

    /* Enter your code after this comment */
    addi sp,sp, -32
    sw ra,0(sp)
    sw x10,4(sp)
    
    li x18,2
    blt x10,x18,_if_

    addi x10,x10,-1
    call asm_fib
    mv x19,x10
    sw x19,8(sp)

    lw x10,4(sp)
    addi x10,x10,-2
    call asm_fib

    mv x20,x10
    lw x19,8(sp)

    add x10,x20,x19
    lw ra,0(sp)
    addi sp,sp,32
    ret
    
    _if_:
      lw x10,4(sp)
      lw ra,0(sp)
      addi sp,sp,32
      ret

    /* Enter your code above this comment */
    ret

.global asm_bsearch
asm_bsearch:
    /* int32_t asm_bsearch(int *arr, int key, 
                    int32_t start,
                    int32_t end)
     * Return the index of the element in the array
     * if the element does not exist, return -1 instead
     * arr: integer array to be searched
     * key: the element we want to search on
     * start: start index of subarray, inclusive
     * end: end index of subarray, inclusiveloop through an entire string and only convert
     **/

    /* Enter your code after this comment */
    addi sp,sp,-32
    sw ra,0(sp)
    sw x10,4(sp)
    sw x11,8(sp)
    sw x12,12(sp)
    sw x13,16(sp)

    bgt x12,x13,_IF_

    add x21,x12,x13
    srai x21,x21,1
    sw x21,20(sp)

    slli x22,x21,2
    add x22,x10,x22
    lw x23,0(x22)
    sw x23,24(sp)

    blt x23,x11,_IF1_
    blt x11,x23,_IF2_

    mv x10,x21
    lw ra,0(sp)
    addi sp,sp,32
    ret

    _IF_:
      li x10,-1
      lw ra,0(sp)
      addi sp,sp,32
      ret

    _IF1_:
      lw x10,4(sp)
      lw x11,8(sp)
      addi x12,x21,1
      lw x13,16(sp)

      call asm_bsearch

      lw ra,0(sp)
      addi sp,sp,32
      ret

    _IF2_:
      lw x10,4(sp)
      lw x11,8(sp)
      lw x12,12(sp)
      lw x13,20(sp)
      addi x13,x13,-1

      call asm_bsearch

      lw ra,0(sp)
      addi sp,sp,32
      ret
    
    /* Enter your code above this comment */
    ret
