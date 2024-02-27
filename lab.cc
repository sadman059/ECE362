#include "lab.h"
#include <stdint.h>
#include <stdio.h>

void setControl(uint32_t instBits, ControlSignals& outputSignals) {
  uint32_t instruc = instBits & 0x7F;
   if (instruc == 0x33) //add,sub,orr
  {
    outputSignals.Branch = Zero;
    outputSignals.MemRead = Zero;
    outputSignals.MemtoReg = Zero;
    outputSignals.ALUOp = 0b10;
    outputSignals.MemWrite = Zero;
    outputSignals.ALUSrc = Zero;
    outputSignals.RegWrite = One;

  }
  //addi,subi
  else if(instruc == 0x13)
  {
    outputSignals.Branch = Zero;
    outputSignals.MemRead = Zero;
    outputSignals.MemtoReg = Zero;
    outputSignals.ALUOp = 0b10;
    outputSignals.MemWrite = Zero;
    outputSignals.ALUSrc = One;
    outputSignals.RegWrite = One;
  }
  //ldur
  else if(instruc == 0x3)
  {
    outputSignals.Branch = Zero;
    outputSignals.MemRead = One;
    outputSignals.MemtoReg = One;
    outputSignals.ALUOp = 0b00;
    outputSignals.MemWrite = Zero;
    outputSignals.ALUSrc = One;
    outputSignals.RegWrite = One;
  }
  //stur
  else if(instruc == 0x23)
  {
    outputSignals.Branch = Zero;
    outputSignals.MemRead = Zero;
    outputSignals.MemtoReg = DC;
    outputSignals.ALUOp = 0b00;
    outputSignals.MemWrite = One;
    outputSignals.ALUSrc = One;
    outputSignals.RegWrite = Zero;
  }
  //cbz
  else if(instruc == 0x63)
  {
    outputSignals.Branch = One;
    outputSignals.MemRead = Zero;
    outputSignals.MemtoReg = DC;
    outputSignals.ALUOp = 0b01;
    outputSignals.MemWrite = Zero;
    outputSignals.ALUSrc = Zero;
    outputSignals.RegWrite = Zero;
  }

}

// Assume that the lower 32-bits of instBits contain the instruction.
uint32_t getExtendedBits(uint32_t instBits) {
  uint32_t returnVal = 0;
  uint32_t instruc = instBits & 0x7F;
  uint32_t temp = instBits >> 20;

  if(instruc == 0x33){
    returnVal = temp;
  }
  else if (instruc == 0x13){
     if(instBits & 0x80000000){
     returnVal = temp | 0xFFFFF000;
     } else{
    returnVal = temp;
    }
  }
  else if (instruc == 0x3){
    returnVal = temp;
  }  
  else if (instruc == 0x23){
    uint32_t inst_7 = instBits >> 7;
    returnVal = (temp & 0xFE0) | (inst_7 & 0x1F);
  }
  else if(instruc == 0x63){
    uint32_t inst_7 = instBits >> 7;
    uint32_t inst_19 = instBits >> 19;
    uint32_t inst_4 = instBits << 4;
    returnVal = (inst_19 & (1<<12)) | (inst_4 & (1<<11)) | (inst_7 & 0x1E) | (temp & 0x3F0);
  }
  return returnVal;
}
