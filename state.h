#pragma once
enum  { NORMAL , SLASH , DOUBLESLASH , SLASHSTAR , STAR , QUOTE , QUOTESPECIAL};
const int STATENUM = 7;
class CommentRemover;

class state{
public:
	virtual void trans(CommentRemover *CR , const char c , char *&target , int &size)=0;
};

class StateNormal : public state{
	virtual void trans(CommentRemover *CR , const char c , char *&target , int &size);
};

class StateSlash : public state{
	virtual void trans(CommentRemover *CR , const char c , char *&target , int &size);
};

class StateDoubleSlash : public state{
	virtual void trans(CommentRemover *CR , const char c , char *&target , int &size);
};

class StateSlashStar : public state{
	virtual void trans(CommentRemover *CR , const char c , char *&target , int &size);
};

class StateStar : public state{
	virtual void trans(CommentRemover *CR , const char c , char *&target , int &size);
};

class StateQuote : public state{
	virtual void trans(CommentRemover *CR , const char c , char *&target , int &size);
};

class StateQuoteSpecial : public state{
	virtual void trans(CommentRemover *CR , const char c , char *&target , int &size);
};
