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
// Modmul RTL
// Das anschließend implementierte Modell ist das RTL
// Modell unseres Modmul Ko-Prozessors
/////////////////////////////////////////////////////

/* Definition der Zustände im Koprozessor*/
`define IDLE	 0
`define LOAD	 1
`define CMP1     2
`define CMP2     3
`define CMP2L    4
`define ERGMULT  5
`define ERGN     6
`define SHLMOD   7
`define CMP3     8
`define CMP3L    9
`define RSUBMOD  10
`define CMP4     11
`define RADDMOD  12
`define CMP5     13
`define SHRMOD   14
`define CMP6     15
`define FINAL    16
`define ERG0     17

/* Beginn des Moduls*/
module ModMul(clk, write, read, addr, data_in, data_out);

  /* Definition der Eingänge in das Modul*/
  input clk;
  input [15:0] data_in;
  input [2:0] addr;
  input write;
  input read;
  
  /*Definiton der Asugänge*/
  output[15:0] data_out;
  
  /* Definition der Rechenvariablen*/
  reg [31:0] zahlA;
  reg [31:0] zahlB;
  reg [31:0] mod;
  reg [15:0] shcnt;
  reg [15:0] cnt;
  reg [31:0] erg;
  reg [15:0] a;
  reg [15:0] b;
  reg [15:0] m;
  reg [1:0] control;
  reg [15:0] data_out;
  
  /*Definition der Verbindungsleitungen vom Kontrollpfad*/
  reg ldall;
  reg lderg;
  reg ergmult;
  reg inccnt;
  reg incshcnt;
  reg shrmod;
  reg shlmod;
  reg ergsubmod;
  reg ergaddmod;
  reg decshcnt;
  
  /*Definition der verschiedenen Zustände die auch im ASM Diagramm vorkommen*/
  `define NUM_STATE_BITS 5
  reg [`NUM_STATE_BITS-1:0] current_state;
  
  /* Dieser Task realisiert den Übergang zwischen Zuständen.
  ** Falls man einen neuen Zustand betreten will, muss man diesen Task ausführen.*/
  task enter_new_state;
    input [`NUM_STATE_BITS-1:0] this_state;
      begin
        #1;
        current_state = this_state;
      end
  endtask
  
  /* RTL Modell des Datenpfades des Modmul Ko-Prozessors*/
  
  /*Zweig für die ZahlA*/
  always @(posedge clk)
    begin
      if (ldall == 1)  //Falls die ZahlA geladen werden soll
        begin
          zahlA <= a;
        end
    end
    
  /*Zweig für die ZahlB*/
  always @(posedge clk)
    begin
      if (ldall == 1)  //Fall die ZahlB geladen werden soll
        begin
          zahlB <= b;
        end
    end
    
  /*Zweig für mod*/
  always @(posedge clk)
    begin
      if(ldall == 1)  //Falls der modulo geladen werden soll
        begin
          mod <= m;
        end
      else if(shlmod == 1)  //Falls der Modulo um 1 nach links geshiftet werden soll
        begin
          mod <= mod << 1;
        end
      else if(shrmod == 1)  //Falls der Modulo um 1 nach rechts geshiftet werden soll
        begin
          mod <= mod >> 1;
        end
    end
  
  /*Zweig für cnt*/
  always @(posedge clk)
    begin
      if(ldall == 1)  //Falls der cnt geladen werden soll
        begin
          cnt <= 0;
        end
      else if(inccnt == 1)  //Falls der cnt erhöht werden soll
        begin
          cnt <= cnt + 1;
        end
    end
    
  /*Zweig für shcnt*/
  always @(posedge clk)
    begin
      if(ldall == 1)  //Falls der shcnt geladen werden soll
	begin
	  shcnt <= 0;
	end
      else if(incshcnt == 1)  //Falls der shcnt erhöht werden soll
	begin
	  shcnt <= shcnt + 1;
	end
      else if(decshcnt == 1)  //Falls der shcnt erniedrigt werden soll
	begin
	  shcnt <= shcnt - 1;
	end
    end
    
  /*Zweig für erg*/
  always @(posedge clk)
    begin
      if(ldall == 1)  //Falls das erg geladen werden soll
        begin
          erg <= m + 1;
        end
      else if(lderg == 1)  //Falls das Ergebnis auf 0 gesetzt werden soll
        begin
          erg <= 0;
        end
      else if(ergmult == 1)  //Falls eine Multiplikation im Ergebnis durchgeführt werden soll
        begin
          erg <= erg + (zahlA << cnt);
        end
      else if(ergsubmod == 1)  //Falls der Modulo vom Ergebnis abgezogen werden soll
        begin
          erg <= erg - mod;
        end
      else if(ergaddmod == 1)  //Falls der Modulo zum Ergebnis addiert werden soll
        begin
          erg <= erg + mod;
        end
    end
    
  /*Datenpfad Ausgänge zum Kontrollpfad*/
  wire zahlAgl0;  //Falls ZahlA gleich o ist
  wire zahlBgl0;  //Falls ZahlB gleich o ist
  wire modgl0;    //Falls mod gleich o ist
  wire cntkl14;   //Falls cnt kl 14 ist
  wire mod30ugl1;  //Falls mod an der Stelle 30 ungleich 1 ist
  wire zahlBcntgl1;  //Falls zahlB an der Stelle cnt gleich 1 ist
  wire erg31gl1;  //Falls erg an der Stelle 31 gleich 1 ist
  wire zahlAklgl32767;  //Falls zahlA kleiner oder gleich 32767 ist
  wire zahlBklgl32767;  //Falls zahlB kleiner oder gleich 32767 ist
  wire modklgl32767;  //Falls mod kleiner oder gleich 32767 ist
  wire shcntugl0;  //Falls shcnt ungleich 0 ist
  wire fin;  //Falls die Moduloreduktion fertig ist
  wire ergklmod;  //Falls das Ergebnis kleiner als der Modul ist
  
  /*Verbidung der Flags mit den Zahlen*/
  assign zahlAgl0 = (zahlA == 0);
  assign zahlBgl0 = (zahlB == 0);
  assign modgl0 = (mod == 0);
  assign cntkl14 = (cnt < 15);
  assign mod30ugl1 = (mod[30] != 1);
  assign zahlBcntgl1 = (zahlB[cnt] == 1);
  assign erg31gl1 = (erg[31] == 1);
  assign zahlAklgl32767 = (zahlA <= 32767);
  assign zahlBklgl32767 = (zahlB <= 32767);
  assign modklgl32767 = (mod <= 32767);
  assign shcntugl0 = (shcnt != 0);
  assign fin = ((mod > erg) && shcnt == 0);
  assign ergklmod = (erg < mod);
  
  /*Output Logik*/
  always @(current_state)
    begin
      control[1] = 0;  //Alle Flags werden zu Beginn auf 0 gesetzt
      ldall = 0;
      lderg = 0;
      ergmult = 0;
      inccnt = 0;
      incshcnt = 0;
      shrmod = 0;
      shlmod = 0;
      ergsubmod = 0;
      ergaddmod = 0;
      decshcnt = 0;
      
      /*Switch Case auf den aktuellen Zustand. Hier wird entschieden welches
      Flag in welchem Zustand gesetzt wird.*/
      case(current_state)
	  `IDLE:  //Falls der aktuelle Zustand IDLE ist
	    begin
	      control[1] = 1;
	    end
	  `LOAD:  //Falls der aktuelle Zustand LOAD ist
	    begin
	      ldall = 1;            
	    end
	  `CMP2:
	    begin
	      lderg = 1;
	    end
	  `ERGMULT:
	    begin
	      ergmult = 1;
	      inccnt = 1;
	    end
	  `ERGN:
	    begin
	      inccnt = 1;
	    end
	  `SHLMOD:
	    begin
	      shlmod = 1;
	      incshcnt = 1;
	    end
	  `RSUBMOD:
	    begin
	      ergsubmod = 1;
	    end
	  `RADDMOD:
	    begin
	      ergaddmod = 1;
	    end
	  `SHRMOD:
	    begin
	      shrmod = 1;
	      decshcnt = 1;
	    end
	  `ERG0:
	    begin
	      lderg = 1;
	    end
      endcase
    end  //always current state
    
  /*Next-State Logik des Ko-Prozessors*/
  reg [`NUM_STATE_BITS-1:0] next_state;  //Define the next_state
  
  // wird immer ausgeführt falls sich eines der Flags oder das 
  // Controllregister ändert
  always @(current_state or zahlAgl0 or zahlBgl0 or modgl0 
           or cntkl14 or mod30ugl1 or zahlBcntgl1 or erg31gl1
           or zahlAklgl32767 or zahlBklgl32767 or modklgl32767
           or shcntugl0 or fin or control[0])
    begin
      case(current_state)
        `IDLE:  //Next state Berechnung für den State IDLE
          begin
            if(control[0] == 1)
              begin
                next_state = `LOAD;
              end
            else
              begin
                next_state =  `IDLE;
              end
          end
        `LOAD:  //Next state Berechnung für den State LOAD
          begin
            next_state = `CMP1;
          end
        `CMP1:  //Next state Berechnung für den State CMP1
          begin
            if(!zahlAgl0 && !zahlBgl0 && !modgl0 && zahlAklgl32767 && zahlBklgl32767 && modklgl32767)
              begin
                next_state = `CMP2;
              end
            else if((zahlAgl0 || zahlBgl0) && !modgl0)
              begin
               next_state = `ERG0;
              end
            else
              begin
                next_state = `IDLE;
              end
          end
        `CMP2:  //Next state Berechnung für den State CMP2
          begin
            next_state = `CMP2L;
          end
        `CMP2L:  //Next state Berechnung für den State CMP2L
          begin
            if(cntkl14)
              begin
                if(zahlBcntgl1)
                  begin
                    next_state = `ERGMULT;
                  end
                else
                  begin
                    next_state = `ERGN;
                  end
              end
            else
              begin
                if(mod30ugl1 && !ergklmod)
                  begin
                    next_state = `SHLMOD;
                  end
                else
                  begin
                    next_state = `CMP3L;
                  end
              end
          end
        `ERGMULT:  //Next state Berechnung für den State ERGMULT
          begin
            next_state = `CMP2L;
          end
        `ERGN:  //Next state Berechnung für den State ERGN
          begin
            next_state = `CMP2L;
          end
        `SHLMOD:  //Next state Berechnung für den State SHLMOD
          begin
            next_state = `CMP3;
          end
        `CMP3:  //Next state Berechnung für den State CMP3
          begin
            next_state = `CMP2L;
          end
        `CMP3L:  //Next state Berechnung für den State CMP3L
	  begin
	    if(fin)
	      begin
		next_state = `IDLE;
	      end
	    else
	      begin
		next_state = `RSUBMOD;
	      end
	  end
        `RSUBMOD:  //Next state Berechnung für den State RSUBMOD
          begin
            next_state = `CMP4;
          end
        `CMP4:  //Next state Berechnung für den State CMP4
          begin
            if(erg31gl1)
              begin
                next_state = `RADDMOD;
              end
            else
              begin
                next_state = `CMP3L;
              end
          end
        `RADDMOD:  //Next state Berechnung für den State RADDMOD
          begin
            next_state = `CMP5;
          end
        `CMP5:  //Next state Berechnung für den State CMP5
          begin
            if(shcntugl0)
              begin
                next_state = `SHRMOD;
              end
            else
              begin
                next_state = `FINAL;
              end
          end
        `SHRMOD:  //Next state Berechnung für den State SHRMOD
          begin
            next_state = `CMP6;
          end
        `CMP6:  //Next state Berechnung für den State CMP6
          begin
            next_state = `CMP3L;
          end
        `FINAL:  //Next state Berechnung für den State FINAL
          begin
            next_state = `IDLE;
          end
        `ERG0:  //Next state Berechnung für den State ERG0
          begin
            next_state = `FINAL;
          end
      endcase
    end
    
    /*Definition des Anfangszustandes*/
    initial
      begin
        current_state = `IDLE;
      end
    
    /*Übernehme den nächsten Zustand bei jeder steigenden Flanke*/
    always @(posedge clk)
      begin
        current_state = next_state;
      end
      
    /* Falls Daten gelesen werden sollen. 
    ** Das Lesen kann Asynchron geschehen, deswegen wird der Ausgang
    ** Immer aktualisiert falls sich eine Zahl, das Controllregister oder
    ** die Adresse ändert */
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
    
  /* Schreibblock
  ** Falls an eine Adresse des Modmul Ko-Prozessors geschrieben
  ** werden soll*/
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
       $write("MA=%h MB=%h PC=%h IR=%h R1=%h R2=%h R3=%h R4=%h R5=%h R6=%h R7=%h R8=%h R9=%h RA=%h RB=%h RC=%h RD=%h RE=%h RF=%h mem[ED]=%h mem[EE]=%h mem[EF]=%h mem[F0]=%h mem[F1]=%h mem[F2]=%h mem[F3]=%h mem[F4]=%h\n",
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
                 toy_cpu_i.R['h7],
                 toy_cpu_i.R['h8],
                 toy_cpu_i.R['h9],
                 toy_cpu_i.R['hA],
                 toy_cpu_i.R['hB],
                 toy_cpu_i.R['hC],
                 toy_cpu_i.R['hD],
                 toy_cpu_i.R['hE],
                 toy_cpu_i.R['hF],
                 m_mem_i.mem['hED],
                 m_mem_i.mem['hEE],
                 m_mem_i.mem['hEF],
                 m_mem_i.mem['hF0],
                 m_mem_i.mem['hF1],
                 m_mem_i.mem['hF2],
                 m_mem_i.mem['hF3],
                 m_mem_i.mem['hF4]
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
	    `IDLE:     $write("IDLE     ");
	    `LOAD:     $write("LOAD     ");
	    `CMP1:     $write("CMP1     ");
	    `CMP2:     $write("CMP2     ");
	    `CMP2L:    $write("CMP2L    ");
	    `ERGMULT:  $write("ERGMULT  ");
	    `ERGN:     $write("ERGN     ");
	    `CMP3:     $write("CMP3     ");
	    `CMP3L:    $write("CMP3L    ");
	    `CMP4:     $write("CMP4     ");
	    `SHLMOD:   $write("SHLMOD   ");
	    `RSUBMOD:  $write("RSUBMOD  ");
	    `CMP5:     $write("CMP5     ");
	    `RADDMOD:  $write("RADDMOD  ");
	    `CMP6:     $write("CMP6     ");
	    `SHRMOD:   $write("SHRMOD   ");
	    `FINAL:    $write("FINAL    ");
	    `ERG0:     $write("ERG0     ");
	  endcase
	  //Das ModMul Modul hat im Idle Zustand noch die Werte aus der vorherigen Berechnung*/
	  $write("start=%b zahlA=%d zahlb=%d mod=%d erg=%d prod=%d ready=%b time=%d\n",
	      modmul.control[0], modmul.a, modmul.b, modmul.m, modmul.erg[15:0], modmul.erg[31:0], modmul.control[1], $time);
        end
      else
        begin
         case(state_code)
	  `IDLE:	   $write("start=%b zahlA=%d zahlb=%d mod=%d erg=%d prod=%d ready=%b time=%d\n",
	      modmul.control[0], modmul.a, modmul.b, modmul.m, modmul.erg[15:0], modmul.erg[31:0], modmul.control[1], $time);
	endcase
        end
    end
  endtask

endmodule
