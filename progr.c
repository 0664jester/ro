#include <stdio.h>

char buchstaben_array[100] = "as0df \t\n";

int R2 = 0;
int R4 = ' ';
int R5 = '\t';
int R6 = '\n';
int R8 = 'EOF';

int R7 = 0; //xor
int R9 = 0; //xor1

int main ()
{
TOP:
  R2=0;
      //einlesen
      goto L1;
  L1: buchstaben_array[R2]=getchar();


						R7 = buchstaben_array[R2]^EOF;
      if(R7==0)
      {
        goto END;
      } goto L2;
		L2:	R7 = buchstaben_array[R2]^R6; //buchstaben_array[R2-1]!=R6
						R2++;
						if(R7 > 0)
						{
								goto L1;
						}

      //reinigen
      R2--;
      R2--;

      goto L5;
  L6: buchstaben_array[R2]=R6;
      R2--;
  L5:	R7 = buchstaben_array[R2]^'b'; //R4
			R9 = buchstaben_array[R2]^'a'; //R5
			if(R7 == 0) goto L6;
			if(R9 == 0) goto L6;

      //ausgeben
      R2=0;
      goto L7;
  L8: putchar(buchstaben_array[R2]);
      R2++;
  L7: R7 = buchstaben_array[R2]^R6;
						R9 = buchstaben_array[R2]^R8;
						if(R7 > 0) if(R9 > 0) goto L8;

						R7 = buchstaben_array[R2]^R8;
						if(R7 == 0)
						{
								putchar(EOF);
								goto END;
						}

						putchar(R6);
     
goto TOP;

  
  
END: putchar(EOF); 
					return 0;
} 
