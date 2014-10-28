#include "processor.h"
#include "factory.h"

Processor *ErrorProcessor::m_pInstance = NULL;
Processor *FolderProcessor::m_pInstance = NULL;
Processor *NotMatchedFileProcessor::m_pInstance = NULL;
Processor *MatchedFileProcessor::m_pInstance = NULL;
Processor *RootFolderProcessor::m_pInstance = NULL;

char *Processor::Connect(const char *path, string addition) const
{
	assert(path);
	string str = (string)path + addition;
	char *ret = new char[str.size()+1];
	strcpy(ret, str.c_str());
	return ret;
}

void NotMatchedFileProcessor::CopyFile_ForLinux(const char *InputPath, const char *OutputPath) const
{
	assert(InputPath && OutputPath);
	copyfile_state_t s;
	s = copyfile_state_alloc();
	copyfile(InputPath, OutputPath, s, COPYFILE_DATA | COPYFILE_XATTR);
}

void FolderProcessor::NewFolder_ForLinux(const char *InputPath, const char *NewPath) const
{
	struct stat statbuf;
	assert(InputPath && lstat(InputPath, &statbuf) >=0);
	mkdir(NewPath, statbuf.st_mode);
}

bool FolderProcessor::IsExistingDirectory_ForLinux(const char *Directory) const
{
	assert(Directory);
	DIR *dir = opendir(Directory);
	return dir != nullptr || access(Directory, R_OK) >= 0;
}

void RootFolderProcessor::DeleteFolder_ForLinux(const char *Directory) const
{
	if (!IsDir_ForLinux(Directory)) 
	{
		return;
	}

	vector<string> contentlist = GetDirectoryDetails_ForLinux(Directory);
	for (string name : contentlist)
	{
		if ("." == name || ".." == name) continue;
		DeleteFolder_ForLinux(Connect(Directory, "/" + name));
	}
}

vector <string> FolderProcessor::GetDirectoryDetails_ForLinux(const char *InputPath) const
{
	struct dirent *dirp;
	DIR *dir = opendir(InputPath);
	assert(dir);
	vector <string> ret;
	while ((dirp = readdir(dir)))
	{
		ret.push_back((string)dirp->d_name);
	}
	return ret;
}

#define CopyFile CopyFile_ForLinux
#define NewFolder NewFolder_ForLinux
#define IsExistingDirectory IsExistingDirectory_ForLinux
#define GetDirectoryDetails GetDirectoryDetails_ForLinux
#define DeleteFolder DeleteFolder_ForLinux

void MatchedFileProcessor::Do(const char *InputPath, const char *OutputPath)
{
	CommentRemover cr;
	cr.Do(InputPath, OutputPath);
}

void NotMatchedFileProcessor::Do(const char *InputPath, const char *OutputPath)
{
	CopyFile(InputPath, OutputPath);
}

void FolderProcessor::Do(const char *InputPath, const char *OutputPath)
{
	vector<string> contentlist = GetDirectoryDetails(InputPath);
	NewFolder(InputPath, OutputPath);
	for (string name : contentlist)
	{
		if ("." == name || ".." == name) continue;
		char *newinputfile = Connect(InputPath, "/" + name);
		char *newoutputfile = Connect(OutputPath, "/" + name);
		ProcessorFactory::CreateProcessor(GetProcessorType(newinputfile))->Do(newinputfile,newoutputfile);
		delete[] newinputfile;
		delete[] newoutputfile;
	}
}

void RootFolderProcessor::Do(const char *InputPath, const char *OutputPath)
{
	if (IsExistingDirectory(OutputPath))
	{
		string str;
		cout << "\"" + (string)OutputPath + "\" is existing. Do you want to cover it?（y or n)" << endl;
		cin >> str;
		if ("y" == str) DeleteFolder(OutputPath);
		else return;
	}
	FolderProcessor::Do(InputPath, OutputPath);
}

void ErrorProcessor::Do(const char *InputPath, const char *OutputPath)
{
	cout << "It's an error when handling with " + (string)InputPath << endl;
}