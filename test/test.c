#include<stdio.h>
#include<sys/types.h>
#include<stdint.h>
#include<stdlib.h>

void Int2Str(int16_t i, uint8_t* str)
{
	uint8_t* p=str;
        uint16_t shifter = i;
	do{
           shifter= shifter/10;
           p++;
       }while(shifter);
       *p='\0';
       do{
	*--p=i%10+48;
         i=i/10;
       }while(i);
}

void Str2Int(uint8_t* str, int16_t* i)
{
        uint8_t j=0;
        uint16_t val=0;
	while(str[j] != '\0')
        {
		val=val*10+str[j++]-48;
        }
        *i=val;
}

int main(int argc, char** argv)
{
  uint8_t buf[8];
  uint16_t i;
  i = atoi(argv[1]);
  Int2Str(i, buf);
  printf("%s \n", buf);
  Str2Int(buf,(int16_t*)&i);
  printf("%d \n", i);
  return 0; 
}
