#include <iostream>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <cassert>
#include <sstream>
#include "CommentRemover.h"
#include "state.h"
using namespace std;

void StateNormal::trans(CommentRemover *CR , const char c , char *&target , int &size)
{
	assert(CR);
	assert(target);
	if ('/' == c)
	{
		CR->ChangeCurrent(SLASH);
	}else if ('\"' == c)
	{
		target[size] = c;
		size ++;
		CR->ChangeCurrent(QUOTE);
	}else
	{
		target[size] = c;
		size ++;
		CR->ChangeCurrent(NORMAL);	
	}
}

void StateSlash::trans(CommentRemover *CR , const char c , char *&target , int &size)
{
	assert(CR);
	assert(target);
	if ('/' == c)
	{
		CR->ChangeCurrent(DOUBLESLASH);
	}else if ('*' == c)
	{
		CR->ChangeCurrent(SLASHSTAR);
	}else
	{
		target[size] = '/';
		target[size+1] = c;
		size += 2;
		CR->ChangeCurrent(NORMAL);
	}
}

void StateDoubleSlash::trans(CommentRemover *CR , const char c , char *&target , int &size)
{
	assert(CR);
	assert(target);
	if ('\n' == c)
	{
		target[size] = '\n';
		size ++;
		CR->ChangeCurrent(NORMAL);
	}else 
	{
		CR->ChangeCurrent(DOUBLESLASH);
	}
}

void StateSlashStar::trans(CommentRemover *CR , const char c , char *&target , int &size)
{
	assert(CR);
	assert(target);
	if ('*' == c)
	{
		CR->ChangeCurrent(STAR);
	}else
	{	 
		CR->ChangeCurrent(SLASHSTAR);
	}	
}

void StateStar::trans(CommentRemover *CR , const char c , char *&target , int &size)
{
	assert(CR);
	assert(target);
	if ('/' == c)
	{
		CR->ChangeCurrent(NORMAL);
	}else if ('*' == c)
	{
		CR->ChangeCurrent(STAR);
	}else
	{
		CR->ChangeCurrent(SLASHSTAR);
	}
}

void StateQuote::trans(CommentRemover *CR , const char c , char *&target , int &size)
{
	assert(CR);
	assert(target);
	if ('\"' == c)
	{
		target[size] = '\"';
		size ++;
		CR->ChangeCurrent(NORMAL);
	}else if ('\\' == c)
	{
		target[size] = c;
		size ++;
		CR->ChangeCurrent(QUOTESPECIAL);
	}else
	{
		target[size] = c;
		size ++;
		CR->ChangeCurrent(QUOTE);
	}
}

void StateQuoteSpecial::trans(CommentRemover *CR , const char c , char *&target , int &size)
{	
	assert(CR);
	assert(target);
	target[size] = c;
	size ++;
	CR->ChangeCurrent(QUOTE);
}