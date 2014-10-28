#include "factory.h"
#include "processor.h"
#include "sys/stat.h"
#include "unistd.h"
#include <cassert>
#include <set>
#include <string>
#include <fstream>
#include <iostream>

#define CAN_Access CanAccess_ForLinux
#define IS_Dir IsDir_ForLinux

set <string> g_ExtensionSet;

void BuildExtensionSet()
{
	ifstream config("config.txt");
	string str;
	while (config.good())
	{
		getline(config, str);
		g_ExtensionSet.insert("." + str);
	}
	config.close();
}

bool MatchExtension(const char *File)
{
	assert(File);
	string str(File);
	int pos = str.rfind(".");
	return pos>=0 && g_ExtensionSet.count(str.substr(pos));	
}

bool CanAccess_ForLinux(const char *InputPath)
{
	assert(InputPath);
	struct stat statbuf;
	return !(lstat(InputPath, &statbuf) < 0 || access(InputPath, R_OK) < 0);	
}

bool IsDir_ForLinux(const char *InputPath)
{
	struct stat statbuf;
	assert(InputPath && lstat(InputPath, &statbuf) >=0);
	return S_ISDIR(statbuf.st_mode);
}

ProcessorType GetProcessorType(const char *InputPath)
{	
	assert(InputPath);
	if (!CAN_Access(InputPath))
	{
		return ERROR;
	}else if (IS_Dir(InputPath)) 
	{
		return FOLDER;
	}else if (MatchExtension(InputPath))
	{
		return MATCHED_FILE; 
	}else
	{
		return NOT_MATCHED_FILE;
	}
}

ProcessorType GetProcessorTypeForRoot(const char *InputPath)
{
	assert(InputPath);
	if (!CAN_Access(InputPath))
	{
		return ERROR;
	}else if (IS_Dir(InputPath))
	{
		return ROOTFOLDER;
	}else if (MatchExtension(InputPath))
	{
		return MATCHED_FILE; 
	}else
	{
		return NOT_MATCHED_FILE;
	}
}

Processor* ProcessorFactory::CreateProcessor(ProcessorType type)
{
	switch (type)
	{
		case ERROR: return ErrorProcessor::GetInstance();
		case FOLDER: return FolderProcessor::GetInstance();
		case MATCHED_FILE: return MatchedFileProcessor::GetInstance();
		case NOT_MATCHED_FILE: return NotMatchedFileProcessor::GetInstance();
		case ROOTFOLDER: return RootFolderProcessor::GetInstance();
	}
}

