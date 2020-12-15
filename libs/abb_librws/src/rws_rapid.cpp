/***********************************************************************************************************************
 *
 * Copyright (c) 2015, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

#include <sstream>
#include <string>

#include "abb_librws/rws_common.h"
#include "abb_librws/rws_rapid.h"

namespace abb
{
namespace rws
{
typedef SystemConstants::RAPID RAPID;

/***********************************************************************************************************************
 * Struct definitions: RAPIDAtomic<RAPIDAtomicTypes>
 */

/************************************************************
 * Auxiliary methods
 */

std::string RAPIDAtomic<RAPID_BOOL>::getType() const
{
  return RAPID::TYPE_BOOL;
}

std::string RAPIDAtomic<RAPID_NUM>::getType() const
{
  return RAPID::TYPE_NUM;
}

std::string RAPIDAtomic<RAPID_DNUM>::getType() const
{
  return RAPID::TYPE_DNUM;
}

std::string RAPIDAtomic<RAPID_STRING>::getType() const
{
  return RAPID::TYPE_STRING;
}

std::string RAPIDAtomic<RAPID_BOOL>::constructString() const
{
  return (value ? RAPID::RAPID_TRUE : RAPID::RAPID_FALSE);
}

std::string RAPIDAtomic<RAPID_NUM>::constructString() const
{
  std::string result("9000000000");

  if (value != (float) 9E9)
  {
    std::stringstream ss;
    ss << value;
    result = ss.str();
  }

  return result;
}

std::string RAPIDAtomic<RAPID_DNUM>::constructString() const
{
  std::string result("9000000000");

  if (value != (float) 9E9)
  {
    std::stringstream ss;
    ss << value;
    result = ss.str();
  }

  return result;
}

std::string RAPIDAtomic<RAPID_STRING>::constructString() const
{
  return "\"" + value + "\"";
}

void RAPIDAtomic<RAPID_BOOL>::parseString(const std::string& value_string)
{
  value = value_string.compare(RAPID::RAPID_TRUE) == 0 ? true : false;
}

void RAPIDAtomic<RAPID_STRING>::parseString(const std::string& value_string)
{
  std::string temp = value_string;

  size_t position = temp.find_first_of("\"");
  if (position == 0)
  {
    temp.erase(position, 1);
  }
  position = temp.find_last_of("\"");
  if (position == temp.size() - 1)
  {
    temp.erase(position, 1);
  }

  std::stringstream ss(temp);
  ss >> value;
}




/***********************************************************************************************************************
 * Class definitions: RAPIDRecord
 */

/************************************************************
 * Primary methods
 */

RAPIDRecord::RAPIDRecord(const std::string& record_type_name)
:
record_type_name_(record_type_name)
{}

std::string RAPIDRecord::getType() const
{
  return record_type_name_;
}

void RAPIDRecord::parseString(const std::string& value_string)
{
  std::vector<std::string> substrings = extractDelimitedSubstrings(value_string);

  if (components_.size() == substrings.size())
  {
    for (size_t i = 0; i < components_.size(); ++i)
    {
      components_.at(i)->parseString(substrings.at(i));
    }
  }
}

std::string RAPIDRecord::constructString() const
{
  std::stringstream ss;

  ss << "[";

  for (size_t i = 0; i < components_.size(); ++i)
  {
    ss << components_.at(i)->constructString();

    if (i != components_.size() - 1)
    {
      ss << ",";
    }
  }

  ss << "]";

  return ss.str();
}

RAPIDRecord& RAPIDRecord::operator=(const RAPIDRecord& other)
{
  if (this != &other)
  {
    if (record_type_name_ == other.record_type_name_)
    {
      if (components_.size() == other.components_.size())
      {
        for (size_t i = 0; i < components_.size(); ++i)
        {
          components_.at(i)->parseString(other.components_.at(i)->constructString());
        }
      }
    }
  }

  return *this;
}

/************************************************************
 * Auxiliary methods
 */

unsigned int RAPIDRecord::countCharInString(std::string input, const char character)
{
  bool done = false;
  unsigned int count = 0;
  size_t position = 0;

  do
  {
    position = input.find_first_of(character);

    if (position != std::string::npos)
    {
      ++count;
      input.erase(position, 1);
    }
    else
    {
      done = true;
    }
  } while (!done);

  return count;
}

std::vector<std::string> RAPIDRecord::extractDelimitedSubstrings(const std::string& input)
{
  // Prepare the input by removing any starting and ending '[' respective ']'
  std::string temp_0(input);
  size_t position_1 = 0;
  size_t position_2 = 0;

  position_1 = temp_0.find_first_of('[');
  position_2 = temp_0.find_last_of(']');

  if (position_1 != std::string::npos && position_2 != std::string::npos)
  {
    temp_0.erase(position_1, 1);
    temp_0.erase(position_2 - 1, 1);
  }

  // Extract and merge the delimited substrings in the prepared input string.
  std::stringstream ss(temp_0);
  std::vector<std::string> values;
  std::string temp_1;
  std::string temp_2;
  int counter = 0;

  while (std::getline(ss, temp_1, ','))
  {
    if (!temp_1.empty())
    {
      counter += countCharInString(temp_1, '[');
      counter -= countCharInString(temp_1, ']');

      if (counter == 0)
      {
        if (!temp_2.empty())
        {
          values.push_back(temp_2 + temp_1);
          temp_2 = "";
        }
        else
        {
          values.push_back(temp_1);
        }
      }
      else
      {
        temp_2 += temp_1 + ",";
      }
    }
  }

  return values;
}

} // end namespace rws
} // end namespace abb
