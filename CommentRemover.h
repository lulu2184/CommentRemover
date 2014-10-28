#pragma once
#include "state.h"

class CommentRemover{
	friend class state;
	friend class StateNormal;
	friend class StateSlash;
	friend class StateDoubleSlash;
	friend class StateSlashStar;
	friend class StateStar;
	friend class StateQuote;
	friend class StateQuoteSpecial;
private:
	int m_osize;
	int m_nsize;
	char *buffer;
	char *content;
	state *current;
	state *State[STATENUM];

	void BuildState();
	void NoInput();
	void ChangeCurrent(const int n);
	bool IsExistingFile(const char *filename);
	void Read(const char *outputfile);
	void Write(const char *outputfile);

public:
	CommentRemover();
	bool IsValidInput(const char *inputfile);
	void Do(const char *inputfile,const char *outputfile);
};