#pragma once
#include <string>
#include <set>
#include "processor.h"
using namespace std;

enum ProcessorType
{
	ERROR = 0,
	MATCHED_FILE,
	NOT_MATCHED_FILE,
	FOLDER,
	ROOTFOLDER
};

extern set <string> g_ExtensionSet;

bool IsDir_ForLinux(const char *InputPath);
void BuildExtensionSet();
bool MatchExtension(const char *File);
ProcessorType GetProcessorType(const char *InputPath);
ProcessorType GetProcessorTypeForRoot(const char *InputPath);

class ProcessorFactory
{
public:
	static Processor* CreateProcessor(ProcessorType type); 
};