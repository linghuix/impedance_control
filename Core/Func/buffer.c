#include <stdio.h> 
#include <stdlib.h>//malloc
#include "buffer.h" 

Buff InitBuffer(uint16_t size)
{
	Buff buffer;
	buffer = (Buff)malloc(sizeof(struct Buffer));
    buffer->data = (ElementType *)malloc(sizeof(ElementType)*size);
	for(int i=0;i<size;i++){
		buffer->data[i]=0;
	}
    buffer->size = size;
    buffer->in = 0; 
	buffer->out = 0;
	printf("size=%-5d buffer->size=%-5d\r\n",size,buffer->size);
	return buffer; 
}

/*
 * author lhx
 * May 13, 2020
 *
 * @brief : put data to buffer, buffer is FULL return 0.
 * Window > Preferences > C/C++ > Editor > Templates.
 */
uint8_t addBuffer(Buff buffer, ElementType pdata)
{
    if((buffer->in+1)%(buffer->size) == buffer->out){
    	printf("full!%d\r\n", pdata);
        return 0;
	}

    buffer->data[buffer->in] = pdata;
    buffer->in = (buffer->in + 1)%(buffer->size);
    return 1;
}

/*
 * author lhx
 * May 13, 2020
 *
 * @brief : get buff data buffer, buffer is EMPty return 0.
 * Window > Preferences > C/C++ > Editor > Templates.
 */
ElementType getBuffer(Buff buffer)
{
    ElementType pdata;
    if(buffer->in == buffer->out)
        return 0;
    pdata = buffer->data[buffer->out++];
    buffer->out = (buffer->out)%(buffer->size);
    return pdata;
}

void print(Buff buffer){
    
    int i;
	int string;
	/*printf("size=%-5d  ",buffer->size);
	printf("in=%-5d  ",buffer->in);
	printf("out=%-5d  ",buffer->out);
	printf("\r\n");*/
    for(i=0;i<buffer->size;i++){
    	printf("%d-",i);
    	string = buffer->data[i];
    	if(i==buffer->in)
    		printf("{in}");
		if(i==buffer->out){
			printf("{out}");
		}
    	//if( string != 0 )
        	printf("%-5d",(string));
    }
    printf("\n");
}


/*  测试代码 */
int main(void)
{
    Buff p;
    ElementType data = 90;
    
    p = InitBuffer(5);//申请空间正好10，再小就会数组越界了
    print(p);
    
    for(int i=1;i<6;i++){
        addBuffer(p ,i);
	}
	print(p);
	printf("get%d\r\n",getBuffer(p));
	printf("get%d\r\n",getBuffer(p));
    
    print(p);
    addBuffer(p ,90);
    addBuffer(p ,900);
    print(p);
    
    return 0;
}
