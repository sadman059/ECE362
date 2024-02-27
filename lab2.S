// Lab 2 template file
.data
.global lowercase_string
lowercase_string:
    // Specify a string ending with '\0'
    .asciz "ece 362 is awesome!"

.global login_name
login_name:
    .asciz "rafi"

.global puid
puid:
    .asciz "0033664036"

// Align to 4bytes boundary as we have a string above
.balign 4

.text
.global asm_strlen
asm_strlen:
    /* uint32_t asm_strlen(char *str)
     * Take in a pointer to a string and return
     * its length up to the null terminating
     * character `\0`
    **/   

    /* Enter your code after this comment */
    li x12,0
    beq x10,zero,End
    lb x11,0(x10)
    
    beq x11,x0,End

    While:
       lb x11,0(x10)
       beq x11,x0,End
       addi x10,x10,1
       addi x12,x12,1
       bne x11,x0,While
    
    End: mv x10,x12
    

    
    /* Enter your code above this comment */
    ret

.global asm_fib
asm_fib:
    /* void asm_fib(int *arr, uint32_t length)
     * Accept a pointer to an array of integer and
     * the length of the array, then generate fibonacci
     * series starting at `0` and store the result
     * back to the array `arr` at corresponding indices.
     **/

    /* Enter your code after this comment */
     beq x10,x0,_End_
     beq x11,x0,_End_

     li x12,0 //Fn-2 term
     li x13,1 //Fn-1 term
     li x14,0 // i = 0


     Loop:
       bge x14,x11,End
       beq x14,x0,Prev
       li x15,1
       beq x14,x15,Curr
       add x6,x13,x12
       mv x12,x13
       mv x13,x6

       sw x6,0(x10)
       jal x0,Incre

    Prev:
       sw x12,0(x10)
       jal x0,Incre

    Curr:
      sw x13,0(x10)

    Incre:
      addi x14,x14,1
      addi x10,x10,4
      jal x0,Loop
    
    _End_:
       ret

     //li x14,4
     //mul x6,x11,x14
     //add x15,x10,x6







    /* Enter your code above this comment */
    ret

.global asm_toUppercase
asm_toUppercase:
    /* void asm_toUppercase(char *str)
     * loop through an entire string and only convert
     * the lowercase letter to uppercase, leaving the
     * rest of the string intact
     **/

    /* Enter your code after this comment */
    beq x10,x0,_End

    _While:
      lb x12,0(x10)
      beq x12,x0,_End

      li x13,97
      blt x12,x13,_Incre

      li x13,122
      blt x13,x12,_Incre

      addi x12,x12,-32

      sb x12,0(x10)

      _Incre:
       addi x10,x10,1
    
    jal x0,_While


    _End:
      ret




    /* Enter your code above this comment */
    ret
