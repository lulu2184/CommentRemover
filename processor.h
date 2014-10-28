#pragma once
#include <iostream>
#include <cstring>
#include <string>
#include <cassert>
#include <set>
#include <cstdlib>
#include <vector>

#include "unistd.h"
#include "copyfile.h"
#include "dirent.h"
#include "sys/stat.h"

#include "CommentRemover.h"
using namespace std;

class Processor
{
protected:
	void ErrorProcessor(const char *path, string printHint) const;
	char *Connect(const char *path, string addition) const;

public:
	Processor(){};
	~Processor(){};
	virtual void Do(const char *InputPath, const char *OutputPath)=0;
};

class MatchedFileProcessor : public Processor
{
private:
	MatchedFileProcessor(){};
	~MatchedFileProcessor(){};
	static Processor *m_pInstance;

public:
	virtual void Do(const char *InputPath, const char *OutputPath);
	static Processor *GetInstance()
	{
		if (NULL == m_pInstance) m_pInstance = new MatchedFileProcessor;
		return m_pInstance;
	}
};

class NotMatchedFileProcessor : public Processor
{
private:
	void CopyFile_ForLinux(const char *InputPath, const char *OutputPath) const;
	NotMatchedFileProcessor(){};
	~NotMatchedFileProcessor(){};
	static Processor *m_pInstance;

public:
	virtual void Do(const char *InputPath, const char *OutputPath);
	static Processor *GetInstance()
	{
		if (NULL == m_pInstance) m_pInstance = new NotMatchedFileProcessor;
		return m_pInstance;
	}
};

class FolderProcessor : public Processor
{
private:
	void NewFolder_ForLinux(const char *InputPath, const char *NewPath) const;
	static Processor *m_pInstance;

protected:
	FolderProcessor(){};
	~FolderProcessor(){};
	bool IsExistingDirectory_ForLinux(const char *Directory) const;
	vector<string> GetDirectoryDetails_ForLinux(const char *InputPath) const;

public:
	virtual void Do(const char *InputPath, const char *OutputPath);
	static Processor *GetInstance()
	{
		if (NULL == m_pInstance) m_pInstance = new FolderProcessor;
		return m_pInstance;
	}
};

class ErrorProcessor : public Processor
{
private:
	ErrorProcessor(){};
	~ErrorProcessor(){};
	static Processor *m_pInstance;

public:
	virtual void Do(const char *InputPath, const char *OutputPath);
	static Processor *GetInstance()
	{
		if (NULL == m_pInstance) m_pInstance = new ErrorProcessor();
		return m_pInstance;
	}
};

class RootFolderProcessor : public FolderProcessor
{
private:
	RootFolderProcessor(){};
	~RootFolderProcessor(){};
	void DeleteFolder_ForLinux(const char *Directory) const;
	static Processor *m_pInstance;

public:
	virtual void Do(const char *InputPath, const char *OutputPath);
	static Processor *GetInstance()
	{
		if (!m_pInstance) m_pInstance = new RootFolderProcessor();
		return m_pInstance;
	}
};

