#include <iostream>
#include "processor.h"
#include "factory.h"
using namespace std;

int main(int argc , char *argv[])
{
	if (argc < 3) 
	{
		printf("usage: ./remover [input directory] [output directory]");
	}
	else
	{
		BuildExtensionSet();
		Processor *pro = ProcessorFactory::CreateProcessor(GetProcessorTypeForRoot(argv[1]));
		pro -> Do(argv[1],argv[2]);
	}

	return 0;
}