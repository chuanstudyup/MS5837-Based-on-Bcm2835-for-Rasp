#include "ms5837.h"

int main()
{

	MS5837 ms5837;
	if(!ms5837.init())
	{
		cout<<"MS5837 init Failed"<<endl;
		exit(-1);
	}
	else
		cout<<"MS5837 init OK"<<endl;
	while(1)
	{
		ms5837.read();
		printf("Depth %fm, Temperature %fdeg\n",ms5837.depth(),ms5837.temperature());
		bcm2835_delay(1000);
	}
	return 0;
}
