/////////////////////////////////////////////////
// KC Posch
// Sep 2010
//
// Version 10.2: LARGE TOY (L-TOY)
//
// Structural model of L-TOY: 
//
// CPU + Memory + I/O module + I/O Modmul
//
// New: 
//   Introducing "interrupt" and a new I/O device acting as a timer
// 
//     New CPU input: interrupt
//     New CPU register: IEN (interrupt enable)
//     Register IEN is set to "off" initially 
//     
//     Interrupt is checked after execution of instruction.
//     If interrupt input is set (and IEF == 1), then
//     value of PC is pushed onto the stack, and 
//     PC gets value 0.
//
//     New I/O device "Timer": 
//      - can issue an interrupt to CPU "every now and then"
//      - can be programmed by CPU:
//               0xFC: control register: bit 1 is interrupt
//                                       bit 0 is on/off
//               0xFD: intervall register: counts clock cycles
//
//      
//
//  - HLT  0x00..  
//  - PUSH 0x01..  
//  - POP  0x02..  
//  - CALL 0x03..  
//  - RET  0x04..
//  - IOF  0x0500   interrupt enable off
//  - ION  0x0600   interrupt enable on
//  - RETI 0x0700
//                 
//
// Machine program: (prog_10_2.v)
//   
//       LD   R2, 0x0F
//       ADD  R2, R2, R1
//       ST   R2, 0x0F
//       ST   R1, 0xFC    // reset interrupt, start counter again   
//       RETI
//       
// main program: initializes values and then runs into endless loop:
//       IOF
//       LDA  RF, 0x50
//       LDA  R1, 1    
//       LDA  R4, 0x7F
//       LDA  R3, 0
//       ST   R0, 0x0F   // initialize interrupt value
//       ST   R4, 0xFD   // intervall = 0x007f
//       ST   R1, 0xFC
//       LDA  R6, 0xFF
//       ION
// loop  ADD  R3, R3, R1
//       SUB  R5, R6, R3 
//       BZ   R5, exit
//       BZ   R0, loop
// exit  HLT 
//
////////////////////////////////////////////////
`define STDIN_FILE  "dummy_in.dat"
`define STDOUT_FILE "dummy_out.dat"

`define NUM_STATE_BITS 6

/////////////////////////////////////////
// define symbolic state names
/////////////////////////////////////////
`define IDLE     0
`define INIT     1
`define FETCH1   2
`define FETCH2   3
`define FETCH3   4
`define DECODE   5

`define HLT   6
`define ADD   7
`define SUB   8
`define AND   9
`define XOR   10
`define SHL   11
`define SHR   12

`define LDA   13

`define LD1   14
`define LD2   15
`define LD3   16

`define ST1   17
`define ST2   18
 
`define LDI1   19
`define LDI2   20
`define LDI3   21

`define STI1   22
`define STI2   23

`define BRZ1   24
`define BRP1   25
`define JR1    26
`define JL1    27

`define PUSH1   28
`define PUSH2   29
`define PUSH3   30

`define POP1    31
`define POP2    32
`define POP3    33

`define CALL1   34
`define CALL2   35
`define CALL3   36

`define RET1    37
`define RET2    38
`define RET3    39

`define INTRPT1 40
`define INTRPT2 41
`define INTRPT3 42

`define ION     43
`define IOF     44

`define RETI1    45
`define RETI2    46
`define RETI3    47

////////////////////////////////////////////////////
// L-TOY CPU
//
//   clk          clock input
//   cont         continue must be set to 1 for 1 clock
//                  cycle in order to start
//                  execution of TOY
//   interrupt    interrupt input
//   cpu_din      data input
//   addr         address output
//   cpu_dout     data output
//   write        output write to memory or I/O
//   read         output read from memory or I/O
////////////////////////////////////////////////////   
module  large_toy_cpu(clk, cont, interrupt, cpu_din, addr, cpu_dout, write, read);
  input  clk;
  input  cont;
  input  interrupt;
  input  [15:0] cpu_din;
  output [7:0] addr;
  output [15:0] cpu_dout;
  output write;
  output read;
  
  wire clk;
  wire cont;
  wire interrupt;
  wire [15:0] cpu_din;

  reg [7:0] MA;
  
  assign addr = MA;
  
  reg [15:0] MB;
  
  assign cpu_dout = MB;
  
  reg [7:0]  PC; 
  reg [15:0] IR;
  reg halt;

  reg IEF;  // interrupt enable flipflop
  
  reg [15:0] R[15:0];
  
  reg [`NUM_STATE_BITS-1:0] present_state;
  reg [`NUM_STATE_BITS-1:0] next_state;

  // no hardware:
  reg [15:0] temp; 

  reg write;
  reg read;
  
  //////////////////////////////////////////////
  // ISE model:
  //////////////////////////////////////////////
  always
    begin
      @(posedge clk) enter_new_state(`INIT);
      PC   <= @(posedge clk) 8'h10;
      halt <= @(posedge clk) 1;
      IEF  <= @(posedge clk) 0;
      while (1)   
        begin
          @(posedge clk) enter_new_state(`FETCH1);
          MA <= @(posedge clk) PC;
          if (halt)
            begin
              while (cont == 0)
                begin
                  @(posedge clk) enter_new_state(`IDLE);
                  halt <= @(posedge clk) 0;
                end // while (~cont)
            end // if (halt)
          else
            begin
              @(posedge clk) enter_new_state(`FETCH2);
              MB <= @(posedge clk) cpu_din; //mem[MA];
              read = 1;
              @(posedge clk) enter_new_state(`FETCH3);
              IR <= @(posedge clk) MB;
              @(posedge clk) enter_new_state(`DECODE);
              PC <= @(posedge clk) PC + 1;
              case (IR[15:12])
                0:  begin
                      case (IR[11:8])
                        0: begin  // 0x0000
                            @(posedge clk) enter_new_state(`HLT);
                            halt <= @(posedge clk) 1;
                           end
                           
                        1: begin  // 0x01..  push // decrement R15; mem[R15] <- R[d]
                            @(posedge clk) enter_new_state(`PUSH1);
                            R[15] <= @(posedge clk) R[15] - 1;
                            @(posedge clk) enter_new_state(`PUSH2);
                            MA <= @(posedge clk) R[15];
                            MB <= @(posedge clk) R[IR[3:0]];
                            @(posedge clk) enter_new_state(`PUSH3);
                            write = 1; 
                           end
                           
                        2: begin  // 0x02..  pop  // R[d] <- mem[R[15]]; increment R15
                            @(posedge clk) enter_new_state(`POP1);
                            MA <= @(posedge clk) R[15];
                            @(posedge clk) enter_new_state(`POP2);
                            R[15] <= @(posedge clk) R[15] + 1;
                            MB <= @(posedge clk) cpu_din; // mem[MA];
                            read = 1;
                            @(posedge clk) enter_new_state(`POP3);
                            R[IR[3:0]] <= @(posedge clk) MB;
                           end

                        3: begin // 0x3..   call  // decrement R15; mem[R15] <- PC; PC <- IR[7:0]
                            @(posedge clk) enter_new_state(`CALL1);
                            R[15] <= @(posedge clk) R[15] - 1;
                            @(posedge clk) enter_new_state(`CALL2);
                            MA <= @(posedge clk) R[15];
                            MB <= @(posedge clk) {8'h0, PC};
                            @(posedge clk) enter_new_state(`CALL3);
                            PC <= @(posedge clk) IR[7:0];
                            write = 1; 
                           end
                           
                        4: begin // 0x4.. ret   // PC <- mem[R15]; increment R15
                            @(posedge clk) enter_new_state(`RET1);
                            MA <= @(posedge clk) R[15];
                            @(posedge clk) enter_new_state(`RET2);
                            R[15] <= @(posedge clk) R[15] + 1;
                            MB <= @(posedge clk) cpu_din; // mem[MA];
                            read = 1;
                            @(posedge clk) enter_new_state(`RET3);
                            PC <= @(posedge clk) MB[7:0];
                           end
                           
                        5: begin  // 0x0500
                            @(posedge clk) enter_new_state(`IOF);
                            IEF <= @(posedge clk) 0;
                           end
                           
                        6: begin  // 0x0600
                            @(posedge clk) enter_new_state(`ION);
                            IEF <= @(posedge clk) 1;
                           end

                        7: begin  // 0x0700  // PC <- mem[R15]; increment R15; IEF <- 1
                            @(posedge clk) enter_new_state(`RETI1);
                            MA <= @(posedge clk) R[15];
                            @(posedge clk) enter_new_state(`RETI2);
                            R[15] <= @(posedge clk) R[15] + 1;
                            MB <= @(posedge clk) cpu_din; // mem[MA];
                            read = 1;
                            @(posedge clk) enter_new_state(`RETI3);
                            PC <= @(posedge clk) MB[7:0];
                            IEF <= @(posedge clk) 1;
                           end

                        default: 
                           begin  // 0x0000
                            @(posedge clk) enter_new_state(`HLT);
                            halt <= @(posedge clk) 1;
                           end
                      endcase
                    end
                    
                1:  begin // R[d] <- R[s] + R[t]
                      @(posedge clk) enter_new_state(`ADD);
                      R[IR[11:8]] <= @(posedge clk) R[IR[7:4]] + R[IR[3:0]];
                    end                
                2: begin // R[d] <- R[s] - R[t]
                      @(posedge clk) enter_new_state(`SUB);
                      R[IR[11:8]] <= @(posedge clk) R[IR[7:4]] - R[IR[3:0]];
                   end
                3: begin // R[d] <- R[s] & R[t]
                      @(posedge clk) enter_new_state(`AND);
                      R[IR[11:8]] <= @(posedge clk) R[IR[7:4]] & R[IR[3:0]];
                   end
                4: begin // R[d] <- R[s] ^ R[t]
                      @(posedge clk) enter_new_state(`XOR);
                      R[IR[11:8]] <= @(posedge clk) R[IR[7:4]] ^ R[IR[3:0]];
                   end
                5: begin // R[d] <- R[s] << R[t]
                      @(posedge clk) enter_new_state(`SHL);
                      R[IR[11:8]] <= @(posedge clk) R[IR[7:4]] << R[IR[3:0]];
                   end
                6: begin // R[d] <- R[s] >> R[t]
                      @(posedge clk) enter_new_state(`SHR);
                      R[IR[11:8]] <= @(posedge clk) right_shift(R[IR[7:4]], R[IR[3:0]]);
                   end
                7: begin // R[d] <- IR[7:0] // load immediate
                      @(posedge clk) enter_new_state(`LDA); 
                      R[IR[11:8]] <= @(posedge clk) IR[7:0];
                   end

                8: begin  // R[d] <- mem[IR[7:0]]  // load direct
                      @(posedge clk) enter_new_state(`LD1);
                      MA <= @(posedge clk) IR[7:0];
                      @(posedge clk) enter_new_state(`LD2);
                      MB <= @(posedge clk) cpu_din; // mem[MA]
                      read = 1;
                      @(posedge clk) enter_new_state(`LD3);
                      R[IR[11:8]] <= @(posedge clk) MB;
                   end
                   
                9: begin  // mem[IR[7:0] <- R[d]  // store direct
                      @(posedge clk) enter_new_state(`ST1);
                      MA <= @(posedge clk) IR[7:0];
                      MB <= @(posedge clk) R[IR[11:8]];
                      @(posedge clk) enter_new_state(`ST2);
                      write = 1; //mem[MA] <= @(posedge clk) MB;
                   end

              'hA: begin // R[d] <- mem[R[t]]    // load indirect
                      @(posedge clk) enter_new_state(`LDI1);
                      MA <= @(posedge clk) R[IR[3:0]];
                      @(posedge clk) enter_new_state(`LDI2);
                      MB <= @(posedge clk) cpu_din; // mem[MA];
                      read = 1;
                      @(posedge clk) enter_new_state(`LDI3);
                      R[IR[11:8]] <= @(posedge clk) MB;
                   end

              'hB: begin // mem[R[t]] <- R[d]    // store indirect
                      @(posedge clk) enter_new_state(`STI1);
                      MA <= @(posedge clk) R[IR[3:0]];
                      MB <= @(posedge clk) R[IR[11:8]];
                      @(posedge clk) enter_new_state(`STI2);
                      write = 1; // mem[MA] <= @(posedge clk) MB;
                   end

              'hC: begin // if(R[d]==0) PC <- IR[7:0]    // branch on zero
                      @(posedge clk) enter_new_state(`BRZ1);
                      if (R[IR[11:8]] == 0)
                        PC <= @(posedge clk) IR[7:0];
                   end
              'hD: begin // if(R[d]>0) PC <- IR[7:0]   // branch on positive
                      @(posedge clk) enter_new_state(`BRP1);
                      if ((R[IR[11:8]] > 0) & (R[IR[11:8]] < 16'h8000)) 
                        PC <= @(posedge clk) IR[7:0];
                   end
              'hE: begin // PC <- R[d]    // jump register
                      @(posedge clk) enter_new_state(`JR1);
                      temp = R[IR[11:8]]; // temp is no hardware; 
                      PC <= @(posedge clk) temp[7:0];
                   end

              'hF: begin // R[d] <- PC, PC <- IR[7:0]   // jump and link
                      @(posedge clk) enter_new_state(`JL1);
                      R[IR[11:8]] <= @(posedge clk) {8'h0,PC};
                      PC <= @(posedge clk) IR[7:0];
                   end

              endcase
            end //else
          if ((interrupt == 1) & (IEF == 1))    // check for interrupt
            begin
              @(posedge clk) enter_new_state(`INTRPT1);
              IEF   <= @(posedge clk) 0;
              R[15] <= @(posedge clk) R[15]-1; 
              @(posedge clk) enter_new_state(`INTRPT2);
              MB <= @(posedge clk) PC;
              PC <= @(posedge clk) 0;
              MA <= @(posedge clk) R[15];
              @(posedge clk) enter_new_state(`INTRPT3);
              write = 1;
            end
        end //while(1)
    end // always

  /////////////////////////////////////////
  // enter_new_state: All output values of 
  // the ISE model are set to 0 at the beginning
  // each period. Then, in the ISE loop above
  // some are set to 1 again, all others stay 0.
  /////////////////////////////////////////
  task enter_new_state;
    input [`NUM_STATE_BITS-1:0] this_state;
      begin
        present_state = this_state;
        #1;
        R[0] = 0;  // R[0] is constant 0
        write = 0;
        read = 0;
      end
  endtask
  
  /////////////////////////////////////////
  // right-shift sign extension
  /////////////////////////////////////////
  function [15:0] right_shift;
    input [15:0] source_reg;
    input [15:0] shift_amount;
    
    begin: serial_shift_with_duplication_of_top_bit
      reg bit_15;
      
      bit_15 = source_reg[15];
      
      repeat(shift_amount)
        begin
          source_reg = (source_reg >> 1) | (bit_15 << 15);
        end
      
      right_shift = source_reg;
    end
  endfunction

endmodule

////////////////////////////////////////////////////
// Main memory (256 x 16)
//
//   clk          clock input
//   mem_din      data input
//   addr         address output
//   mem_dout     data output
//   write        input write
//
////////////////////////////////////////////////////   
module main_memory(clk, write, addr, mem_din, mem_dout);
  input clk;
  input write;
  input [7:0] addr;
  input [15:0] mem_din;
  output [15:0] mem_dout;
  
  reg [15:0] mem[0:'hFF];

  wire [7:0] addr;
  wire [15:0] mem_din;
  reg [15:0] mem_dout;
  
  // main memory reading :
  always @(addr)
    mem_dout = mem[addr];
  
  // main memory writing:
  always @(posedge clk)
    begin
      if (write)
        mem[addr] <= mem_din;
    end

endmodule

/////////////////////////////////////////////////////
// IO-module. 
//   Use this module for connecting the TOY cpu
//   to an input file which defines a sequence of
//   input values, and an output file to be created
//   by writing to address 0xFF.
//
// Interface:
//   clk       clock input
//   read      control input. If read is set to 1,
//               the next 16-bit value from the
//               input file is output at data_out.
//   write     control input. If write is set to 1,
//               the 16-bit value at input data_in
//               is appended to the file STD_OUT_FILE
//   addr      1-bit address in order to distinguish 
//               control register CR (addr = 0) and 
//               data register DR (addr = 1)
//   data_in   16-bit data input
//   data_out  16-bit data output
//
// Specialties: 
//   The control input "read" can be set to 1 for
//   several consecutive clock cycles. This will
//   result in only one write activity to the file.
//   
/////////////////////////////////////////////////////
`define NUM_IO_STATE_BITS 5

`define IO_INIT           0
`define IO_SET_DR_AND_CR  1
`define IO_DELAY_ONE      2
`define IO_WAIT_FOR_CPU   3
`define IO_INCR_PTR       4
`define IO_DELAY_COUNTER  5
`define IO_DELAY_CNTR_INIT 6
`define IO_END            7


module io(clk, read, write, addr, data_in, data_out);
  input  clk, read, write;
  input  addr;
  input [15:0] data_in;
  output [15:0] data_out;

  wire clk, read, write;
  wire addr;
  wire [15:0] data_in;
  
  reg [15:0] data_out;
  
  reg [15:0] delay_counter; // for slowing down the in_out module;

  integer i;
  integer std_out_handle; // file handle
  
  reg [15:0] std_in[0:1023];
  reg [9:0] std_in_pointer;
  
  reg write_delayed;

  reg CR;
  reg [15:0] DR;

  reg [`NUM_IO_STATE_BITS-1:0] io_present_state;
  
  initial
    begin
      // load data for std_in:
      $readmemh(`STDIN_FILE, std_in);

      // for convenience: print the values
      // just read from the input file "std_in.dat"
      for (i=0; i<20; i = i+1)
          $display("st_din[%d]=%h", i, std_in[i]);
      
      std_out_handle = $fopen(`STDOUT_FILE);
    end
    
  always
    begin
      @(posedge clk) enter_new_state(`IO_INIT);
      std_in_pointer <= @(posedge clk) 0;
      CR <= @(posedge clk) 0;
      delay_counter <= @(posedge clk) 0;
      while(1)
        begin
          @(posedge clk) enter_new_state(`IO_SET_DR_AND_CR);  
          DR <= @(posedge clk) std_in[std_in_pointer];
          CR <= @(posedge clk) 1;
          @(posedge clk) enter_new_state(`IO_DELAY_ONE);  
          while (CR == 1)
            @(posedge clk) enter_new_state(`IO_WAIT_FOR_CPU); 
          @(posedge clk) enter_new_state(`IO_INCR_PTR); 
          std_in_pointer <= @(posedge clk) std_in_pointer + 1;
          while (delay_counter < 10)
            begin
              @(posedge clk) enter_new_state(`IO_DELAY_COUNTER); 
              delay_counter <= @(posedge clk) delay_counter + 1;
            end
          @(posedge clk) enter_new_state(`IO_DELAY_CNTR_INIT); 
          delay_counter <= @(posedge clk) 0;
        end
      while(1)
        @(posedge clk) enter_new_state(`IO_END);
    end
     
  always @(DR or CR or addr)
    if (addr == 0)
      data_out = CR;
    else if (addr == 1)
      data_out = DR;
    
  always @(posedge clk)
    if (write == 1)
      begin
        if (addr == 0)
          CR <= data_in[0];
        else if (addr == 1)
          // write to file whenever the cpu writes to address 0xFF:
          $fdisplay(std_out_handle, "%h", data_in);
      end

  /////////////////////////////////////////
  // enter_new_state: All output values of 
  // the ISE model are set to 0 at the beginning
  // each period. Then, in the ISE loop above
  // some are set to 1 again, all others stay 0.
  /////////////////////////////////////////
  task enter_new_state;
    input [`NUM_IO_STATE_BITS-1:0] io_this_state;
      begin
        io_present_state = io_this_state;
        #1;
      end
  endtask

endmodule



/////////////////////////////////////////////////////
// Modmul 
/////////////////////////////////////////////////////

/*Definition der verschiedenen Zustände die auch im ASM Diagramm vorkommen*/
`define NUM_STATE_BITS 4
`define IDLE             0
`define LOAD             1
`define CMP1     2
`define CMP2     3
`define SHLMULT  4
`define CMP3     5
`define CMP3L    6
`define CMP4     7
`define CMP4L    8
`define SHLMOD   9
`define RSUBMOD  10
`define CMP5     11
`define RMODADD  12
`define CMP6     13
`define SHRMOD   14
`define FINAL    15
  
module ModMul(clk, write, read, addr, data_in, data_out);

  /* Definition der Eingänge in das Modul*/
  input clk;
  
  input [15:0] data_in;
  input [2:0] addr;
  input write;
  input read;
  
  output [15:0] data_out;

  /* Definition der Rechenvariablen*/
  reg [31:0] zahlA;
  reg [31:0] zahlB;
  reg [31:0] mod;
  reg [15:0] shcnt;
  reg [15:0] cnt;
  reg [31:0] erg;
  reg fin;
  reg [15:0] a;
  reg [15:0] b;
  reg [15:0] m;
  reg [1:0] control;
  reg [15:0] data_out;
  reg [`NUM_MODMUL_STATE_BITS-1:0] current_state;
  
  /* Dieser Task realisiert den Übergang zwischen Zuständen.
  ** Falls man einen neuen Zustand betreten will, muss man diesen Task ausführen.
  ** In ihm wird auch das ready Flag immer auf 0 gesetzt. Will man dieses in einem Zustand auf 1 setzen,
  ** so muss man dies explizit hinschreiben.*/
  task enter_new_state;
    input [`NUM_MODMUL_STATE_BITS-1:0] this_state;
      begin
        #1;
        current_state = this_state;
        control[1] = 0;
      end
  endtask

  initial a = 0;
  initial b = 0;
  initial m = 0;
  initial control = 0;

  /* Alwaysblock des ModMul moduls*/
  always
    begin
      @(posedge clk) enter_new_state(`IDLE);            //Beginne im IDLE Zustand
      control[1] = 1;                                        //Setze das Flag ready auf 1
      if(control[0])
        begin
        @(posedge clk) enter_new_state(`LOAD);        //Falls start gesetzt ist, gehe in den Zustand LOAD
        control[1] = 0;
        fin = 0;                                      //Setze das Flag, welches intern anzeigt ob die Berechnugn fertig ist auf 0
        zahlA <= @(posedge clk) a;                    //Initialisiere die Variablen
        zahlB <= @(posedge clk) b;
        mod <= @(posedge clk) m;
        shcnt <= @(posedge clk) 0;
        cnt <= @(posedge clk)  0;
        erg <= @(posedge clk) m+1;                    //Setze das Ergebnis auf einen ungültigen Wert, damit auf Fehler überprüft werden kann
        @(posedge clk) enter_new_state(`CMP1);        //Gehe in den Zustand CMP1
        if(zahlA > 0 && zahlB > 0 && mod > 0 && zahlA <= 32767 && zahlB <= 32767 && mod <= 32767) //Überprüfe die Variablen auf ihre Gültigkeit
        begin
          @(posedge clk) enter_new_state(`CMP2);  //Falls alle Variablen gültig sind, gehe in Zustand CMP2
          erg <= @(posedge clk) 0;                //Setze das Ergebnis wieder auf 0
          
          while(cnt < 14)                          //Multiplikation zweier Zahlen
            begin 
              @(posedge clk) enter_new_state(`SHLMULT); //Wenn der cnt noch nicht den Wert 6 erreicht hat, gehe in den Zustand SHLMULT
              if(zahlB[cnt] == 1)  //Überprüfe ob das Bit an der Stelle cnt der zahlB 1 ist 
                begin
                  erg <= @(posedge clk) erg + (zahlA << cnt); //Führe die Shift and Add Methode durch
                end
              else
                begin
                  erg <= @(posedge clk) erg;  //Falls das Bit in zahlB nicht 1 war, verändere das Ergebnis nicht
                end
              cnt <= @(posedge clk) cnt + 1;  //Zum Schluss erhöhe die cnt Variable
            end //end while
      
          @(posedge clk) enter_new_state(`CMP3); //Wenn die Mutliplikation abgeschlossen ist, gehe in den Zustand CM3
          while(mod[30] != 1)  //Führe die Schleife aus, solange das zweithöchste Bit in mod nicht 1 ist
            begin
              @(posedge clk) enter_new_state(`SHLMOD); //Gehe immer in den Zustand SHLMOD
              mod <= @(posedge clk) mod << 1;          //Shifte den mod um 1 nach links
              shcnt <= @(posedge clk) shcnt + 1;       //erhöhe den shcnt um 1
              @(posedge clk) enter_new_state(`CMP3L);  //Gehe in den CMP3L Zustand 
            end //while
          
          @(posedge clk) enter_new_state(`CMP4);       //Nachdem der Modulo modifiziert wurde, gehe in den Zustand CMP4
          while(fin == 0)                              //Führe die Schleife aus, solange die Berechnung noch nicht abgeschlossen ist
            begin
              @(posedge clk) enter_new_state(`RSUBMOD); //Gehe in den Zustand RSUBMOD
              erg <= @(posedge clk) erg - mod;          //Ziehe den Modulo vom Ergebnis ab
              
              @(posedge clk) enter_new_state(`CMP5);
              if(erg[31] == 1)                           //check if the erg register is < 0
                begin
                @(posedge clk) enter_new_state(`RMODADD); //Gehe in den Zustand RMODADD
                erg <= @(posedge clk) erg + mod;     //Falls das Ergebnis negativ wurde, addieren wieder den Modulo
                if(shcnt != 0)                       //Falls der shcnt bereits 0 ist
                  begin
                    @(posedge clk) enter_new_state(`SHRMOD); //Gehe in den Zustand SHRMOD
                    mod <= @(posedge clk) mod >> 1;          //shifte den Modulo um 1 nach rechts
                    shcnt <= @(posedge clk) shcnt -1;        //verringere shcnt um 1
                    @(posedge clk) enter_new_state(`CMP4L);
                  end
                else                                         //Falls shcnt gleich 0 ist 
                begin
                  @(posedge clk) enter_new_state(`FINAL);   //Gehe in den Zustand FINAL
                  fin = 1;                                  //setze das Flag fin um aus der Schleife auszusteigen
                end
              end //end if erg < 0
            end //end while
      end //end if CMP1
    else if((zahlA == 0 || zahlB == 0) && mod != 0)         //Falls eine der beiden Zahlen 0 ist, ist das Ergebnis sicher 0
      begin
        erg <= @(posedge clk) 0;
      end
        end //end if Start
    end

  always @(a or b or m or control or addr)
    begin
      if(addr == 3)
        begin
          data_out = erg[15:0];
        end
      else if(addr == 4)
        begin
          data_out = control;
        end
    end
    
  always @(posedge clk)
    begin
      if (write)
        begin
        case (addr)
          0: a       <= data_in;
          1: b       <= data_in;
          2: m       <= data_in;
          4: control <= data_in[1:0];
        endcase
        end
    end
endmodule


////////////////////////////////////////////////////
// Testbench top:
//
// Module "top" connects instances of the three modules
// defined above:
//    - toy_cpu
//    - mem
//    - io
//    - timer
//
// In addition, it contains decoding logic
// used in oder to distinguish between memory addreaddr[2:0]sses,
// io-addresses, and timer addresses 
// Addresses 0 through to 0xFB are memory addresses,
// 0xFC und 0xFD are timer adresses, 
// and 0xFE and 0xFF are connected to the input-output module.
//
//   starts a clock generator,
//   creates an instance of the module toy
//   stops simulation when cpu reaches halt,
//   prints a debug message for each clock cycle.
//   
////////////////////////////////////////////////////
module top;
  reg clk, continue;

  wire [15:0] cpu_din;
  wire [7:0] addr;
  wire [15:0] cpu_dout;
  
  wire write, read;
  
  wire write_io, write_mem, read_io, write_modulo, read_modulo;
  wire [15:0] mem_dout, io_dout, mod_mul_out;
  
  wire interrupt;
  
  integer i;
  
  initial clk = 0;
  always  #50 clk = ~clk;

  // instantiation of cpu:
  large_toy_cpu toy_cpu_i(clk, continue, interrupt, cpu_din, addr, cpu_dout, write, read);

  // decoding the write signal:
  assign write_mem   = write & (addr < 16'hFC);
  assign write_io    = write & (addr >= 16'hFE);
  assign write_modulo = write & ((addr == 16'hF0) | (addr == 16'hF1) | (addr == 16'hF2) | (addr == 16'hF4));

  // decoding the read signal:
  assign read_io   = read  & (addr >= 16'hFE);
  assign read_modulo = read & ((addr == 16'hF3) | (addr == 16'hF4));
  
  // multiplexer for TOY's data input: data input
  // is taken either from input-output or from memory:
  assign cpu_din = (addr >= 16'hF0) ? ((addr >= 16'hF0 && addr <= 16'hF7) ? mod_mul_out : io_dout) : mem_dout;

  // instantiation of main memory:
  main_memory m_mem_i(clk, write_mem, addr, cpu_dout, mem_dout);

  // instantiation of io-module:
  io in_out(clk, read_io, write_io, addr[0], cpu_dout, io_dout);
  
  ModMul modmul(clk, write_modulo, read_modulo, addr[2:0], cpu_dout, mod_mul_out);


  initial   
    begin
      // define contents of main memory:

      /* Software which runs on the toy */
      m_mem_i.mem['h0000] = 'h7FFF;
      m_mem_i.mem['h0010] = 'h7110;
      m_mem_i.mem['h0011] = 'h7200;
      m_mem_i.mem['h0012] = 'h73ED;
      m_mem_i.mem['h0013] = 'h7401;
      m_mem_i.mem['h0014] = 'h7600;
      m_mem_i.mem['h0015] = 'h7703;
      m_mem_i.mem['h0016] = 'h7802;
      m_mem_i.mem['h0017] = 'h7900;
      m_mem_i.mem['h0018] = 'h73ED;
      m_mem_i.mem['h0019] = 'h7600;
      m_mem_i.mem['h001A] = 'h7703;
      m_mem_i.mem['h001B] = 'h7F01;
      m_mem_i.mem['h001C] = 'h8BFE;
      m_mem_i.mem['h001D] = 'hCB1C;
      m_mem_i.mem['h001E] = 'h8AFF;
      m_mem_i.mem['h001F] = 'h90FE;
      m_mem_i.mem['h0020] = 'hCA49;
      m_mem_i.mem['h0021] = 'h8200;
      m_mem_i.mem['h0022] = 'h22A2;
      m_mem_i.mem['h0023] = 'hD249;
      m_mem_i.mem['h0024] = 'hBA03;
      m_mem_i.mem['h0025] = 'h1334;
      m_mem_i.mem['h0026] = 'h2774;
      m_mem_i.mem['h0027] = 'hD71C;
      m_mem_i.mem['h0028] = 'h73ED;
      m_mem_i.mem['h0029] = 'hAC03;
      m_mem_i.mem['h002A] = 'h1334;
      m_mem_i.mem['h002B] = 'hAD03;
      m_mem_i.mem['h002C] = 'h1334;
      m_mem_i.mem['h002D] = 'hAE03;
      m_mem_i.mem['h002E] = 'h1334;
      m_mem_i.mem['h002F] = 'h2261;
      m_mem_i.mem['h0030] = 'hC247;
      m_mem_i.mem['h0031] = 'h65D6;
      m_mem_i.mem['h0032] = 'h3254;
      m_mem_i.mem['h0033] = 'hD23B;
      m_mem_i.mem['h0034] = 'h9CF0;
      m_mem_i.mem['h0035] = 'h9CF1;
      m_mem_i.mem['h0036] = 'h9EF2;
      m_mem_i.mem['h0037] = 'hF941;
      m_mem_i.mem['h0038] = 'h8CF3;
      m_mem_i.mem['h0039] = 'h1664;
      m_mem_i.mem['h003A] = 'hC02F;
      m_mem_i.mem['h003B] = 'h9FF0;
      m_mem_i.mem['h003C] = 'h9CF1;
      m_mem_i.mem['h003D] = 'h9EF2;
      m_mem_i.mem['h003E] = 'hF941;
      m_mem_i.mem['h003F] = 'h8FF3;
      m_mem_i.mem['h0040] = 'hC034;
      m_mem_i.mem['h0041] = 'h94F4;
      m_mem_i.mem['h0042] = 'h90F4;
      m_mem_i.mem['h0043] = 'h82F4;
      m_mem_i.mem['h0044] = 'h3228;
      m_mem_i.mem['h0045] = 'hC243;
      m_mem_i.mem['h0046] = 'hE900;
      m_mem_i.mem['h0047] = 'h9FFF;
      m_mem_i.mem['h0048] = 'hC018;
      m_mem_i.mem['h0049] = 'h0000;

      $display("Memory contents before execution:");
      for (i=0; i< 'h10; i=i+1)
        $display("mem[%0h] = %h \t mem[%0h] = %h \t mem[%0h] = %h \t mem[%0h] = %h \t mem[%0h] = %h \t mem[%0h] = %h", 
                  i, m_mem_i.mem[i], 
                  i+16, m_mem_i.mem[i+16],
                  i+32, m_mem_i.mem[i+32],
                  i+48, m_mem_i.mem[i+48],
                  i+64, m_mem_i.mem[i+64],
                  i+240, m_mem_i.mem[i+240]);

      // start the cpu by pressing continue:
      continue = 0;
      #500 continue = 1;
      #100 continue = 0;

      // let cpu run and wait until halt is set:
      wait (toy_cpu_i.halt);
      
      //#80000

      $fclose(in_out.std_out_handle);
      
      $display("\nMemory contents after execution:");
      for (i=0; i< 'h10; i=i+1)
        $display("mem[%0h] = %h \t mem[%0h] = %h \t mem[%0h] = %h \t mem[%0h] = %h \t mem[%0h] = %h \t mem[%0h] = %h", 
                  i, m_mem_i.mem[i], 
                  i+16, m_mem_i.mem[i+16],
                  i+32, m_mem_i.mem[i+32],
                  i+48, m_mem_i.mem[i+48],
                  i+64, m_mem_i.mem[i+64],
                  i+240, m_mem_i.mem[i+240]);

      $finish;
  end
  
  /////////////////////////////////////////
  // debug output at each clock cycle
  /////////////////////////////////////////
   always @(posedge clk) #9
     begin
       if (toy_cpu_i.present_state == `FETCH1)
        $display("%d----------------------------------------------------------------------", $time);
       print_state_name(toy_cpu_i.present_state);
       $write("MA=%h MB=%h PC=%h IR=%h R1=%h R2=%h R3=%h R4=%h R5=%h R6=%h        RF=%h IEF=%b IO=%b address=%b \n",
                 toy_cpu_i.MA,
                 toy_cpu_i.MB,
                 toy_cpu_i.PC,
                 toy_cpu_i.IR,
                 toy_cpu_i.R['h1],
                 toy_cpu_i.R['h2],
                 toy_cpu_i.R['h3],
                 toy_cpu_i.R['h4],
                 toy_cpu_i.R['h5],
                 toy_cpu_i.R['h6],
                 toy_cpu_i.R['hF],
                 toy_cpu_i.cpu_din,
                 in_out.CR,
                 addr
              );
        printDebugInfo(modmul.current_state, 1);   
        //print_timer_state_name(timer_i.timer_present_state);
        $write("\n");
     end

       
  /////////////////////////////////////////
  // print_state_name: no hardware, task
  // is used to produce nice output on console
  /////////////////////////////////////////
  task print_state_name;
    input [`NUM_STATE_BITS-1:0] state_code;
    begin
      case(state_code)
        `INIT:    $write("INIT     ");
        `IDLE:    $write("IDLE     ");
        `FETCH1:  $write("FETCH1   ");
        `FETCH2:  $write("FETCH2   ");
        `FETCH3:  $write("FETCH3   ");
        `DECODE:  $write("DECODE   ");
       
        `HLT:     $write("HLT      ");
        `ADD:     $write("ADD      ");
        `SUB:     $write("SUB      ");
        `AND:     $write("AND      ");
        `XOR:     $write("XOR      ");
        `SHL:     $write("SHL      ");
        `SHR:     $write("SHR      ");

        `LDA:     $write("LDA      ");

        `LD1:     $write("LD1      ");
        `LD2:     $write("LD2      ");
        `LD3:     $write("LD3      ");
       
        `ST1:     $write("ST1      ");
        `ST2:     $write("ST2      ");

        `LDI1:    $write("LDI1     ");
        `LDI2:    $write("LDI2     ");
        `LDI3:    $write("LDI3     ");

        `STI1:    $write("STI1     ");
        `STI2:    $write("STI2     ");

        `BRZ1:    $write("BRZ1     ");
        `BRP1:    $write("BRP1     ");
        `JR1:     $write("JR1      ");
        `JL1:     $write("JL1      ");

        `PUSH1:   $write("PUSH1    ");
        `PUSH2:   $write("PUSH2    ");
        `PUSH3:   $write("PUSH3    ");

        `POP1:    $write("POP1     ");
        `POP2:    $write("POP2     ");
        `POP3:    $write("POP3     ");

        `CALL1:   $write("CALL1    ");
        `CALL2:   $write("CALL2    ");
        `CALL3:   $write("CALL3    ");

        `RET1:    $write("RET1     ");
        `RET2:    $write("RET2     ");
        `RET3:    $write("RET3     ");

        `INTRPT1: $write("INTRPT1  ");
        `INTRPT2: $write("INTRPT2  ");
        `INTRPT3: $write("INTRPT3  ");

        `ION:     $write("ION      ");
        `IOF:     $write("IOF      ");

        `RETI1:   $write("RETI1    ");
        `RETI2:   $write("RETI2    ");
        `RETI3:   $write("RETI3    ");

      endcase
    end
  endtask


  /////////////////////////////////////////
  // print_io_state_name: no hardware, task
  // is used to produce nice output on console
  /////////////////////////////////////////
  task print_io_state_name;
    input [`NUM_IO_STATE_BITS-1:0] io_state_code;
    begin
      case(io_state_code)
        `IO_INIT:            $write("IO_INIT            ");
        `IO_SET_DR_AND_CR:   $write("IO_SET_DR_AND_CR   ");
        `IO_DELAY_ONE:       $write("IO_DELAY_ONE       ");
        `IO_WAIT_FOR_CPU:    $write("IO_WAIT_FOR_CPU    ");
        `IO_INCR_PTR:        $write("IO_INCR_PTR        ");
        `IO_DELAY_COUNTER:   $write("IO_DELAY_COUNTER   ");
        `IO_DELAY_CNTR_INIT: $write("IO_DELAY_CNTR_INIT" );
        `IO_END:             $write("IO_END             ");
      endcase
    end
  endtask
  
  task printDebugInfo;
    input [`NUM_STATE_BITS-1:0] state_code;
    input [7:0] depth;
    begin
      if(depth == 1)
        begin
          case(state_code)
              `IDLE:     $write("MODMUL: IDLE     ");
              `LOAD:     $write("MODMUL: LOAD     ");
              `CMP1:     $write("MODMUL: CMP1     ");
              `CMP2:     $write("MODMUL: CMP2     ");
              `SHLMULT:  $write("MODMUL: SHLMULT  ");
              `CMP3:     $write("MODMUL: CMP3     ");
              `CMP3L:    $write("MODMUL: CMP3L    ");
              `CMP4:     $write("MODMUL: CMP4     ");
              `CMP4L:    $write("MODMUL: CMP4L    ");
              `SHLMOD:   $write("MODMUL: SHLMOD   ");
              `RSUBMOD:  $write("MODMUL: RSUBMOD  ");
              `CMP5:     $write("MODMUL: CMP5     ");
              `RMODADD:  $write("MODMUL: RMODADD  ");
              `CMP6:     $write("MODMUL: CMP6     ");
              `SHRMOD:   $write("MODMUL: SHRMOD   ");
              `FINAL:    $write("MODMUL: FINAL    ");
            endcase
            //Das ModMul Modul hat im Idle Zustand noch die Werte aus der vorherigen Berechnung*/
            $write("start=%b zahlA=%d zahlb=%d mod=%d erg=%d prod=%d ready=%b time=%d\n",
                modmul.control[0], modmul.a, modmul.b, modmul.m, modmul.erg[15:0], modmul.erg[31:0], modmul.control[1], $time);
        end
      else
        begin
         case(state_code)
            `IDLE: $write("start=%b zahlA=%d zahlb=%d mod=%d erg=%d prod=%d ready=%b time=%d\n",
                modmul.control[0], modmul.a, modmul.b, modmul.m, modmul.erg[15:0], modmul.erg[31:0], modmul.control[1], $time);
          endcase
        end
    end
  endtask

endmodule
