#pragma once
#include "ofMain.h"
#include <cstring>
#include <string>

void urdfStringSplit(std::vector<std::string>& pieces, const std::string& vector_str, const std::vector<std::string>& separators);

void urdfIsAnyOf(const char* seps, std::vector<std::string>& strArray);

#ifdef __cplusplus
extern "C"
{
#endif

	///The string split C code is by Lars Wirzenius
	///See http://stackoverflow.com/questions/2531605/how-to-split-a-string-with-a-delimiter-larger-than-one-single-char

	/* Split a string into substrings. Return dynamic array of dynamically
 allocated substrings, or NULL if there was an error. Caller is
 expected to free the memory, for example with str_array_free. */
	char** urdfStrSplit(const char* input, const char* sep);

	/* Free a dynamic array of dynamic strings. */
	void urdfStrArrayFree(char** array);

	/* Return length of a NULL-delimited array of strings. */
	size_t urdfStrArrayLen(char** array);

#ifdef __cplusplus
}
#endif

