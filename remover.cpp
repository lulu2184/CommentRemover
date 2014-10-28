#include <iostream>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <cassert>
#include "CommentRemover.h"
#include "state.h"
using namespace std;

CommentRemover::CommentRemover()
{
	BuildState();
	m_nsize = 0;
	current = State[NORMAL];
}

void CommentRemover::BuildState()
{
	State[NORMAL] = new StateNormal;
	State[SLASH] = new StateSlash;
	State[DOUBLESLASH] = new StateDoubleSlash;
	State[SLASHSTAR] = new StateSlashStar;
	State[STAR] = new StateStar;
	State[QUOTE] = new StateQuote;
	State[QUOTESPECIAL] = new StateQuoteSpecial;
}

void CommentRemover::Read(const char *inputfile)
{
	assert(inputfile);
	ifstream ifile(inputfile);
	ifile.seekg(0,ifile.end);
	m_osize = ifile.tellg();
	ifile.seekg(0);
	buffer = new char[m_osize];
	ifile.read(buffer,m_osize);
	content = new char[m_osize];
}

bool CommentRemover::IsExistingFile(const char *filename)
{
	assert(filename);
	fstream file;
	file.open(filename,ios::in);
	return file.is_open();
}

bool CommentRemover::IsValidInput(const char *inputfile)
{
	return inputfile && IsExistingFile(inputfile);
}

void CommentRemover::ChangeCurrent(const int n)
{
	if (n >= 0 && n <= STATENUM) 
	{	
		current = State[n];
	}else 
	{
		printf("Invalid position in State.");
	}
}

void CommentRemover::Do(const char *inputfile , const char *outputfile)
{
	assert(inputfile);
	assert(outputfile);
	Read(inputfile);
	for (int i = 0 ; i < m_osize ; i++)
		current -> trans(this,buffer[i],content,m_nsize); 
	Write(outputfile);
}

void CommentRemover::Write(const char *outputfile)
{
	assert(outputfile);
	ofstream ofile(outputfile);
	ofile.write(content,m_nsize);
	ofile.close();
}