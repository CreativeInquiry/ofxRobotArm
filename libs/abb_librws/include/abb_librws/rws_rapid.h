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

#ifndef RWS_RAPID_H
#define RWS_RAPID_H

#include <string>
#include <vector>
#include <sstream>

#include "Poco/SharedPtr.h"

namespace abb
{
namespace rws
{
/**
 * \brief An enum for different RAPID atomic data types.
 */
enum RAPIDAtomicTypes
{
  RAPID_BOOL,  ///< \brief RAPID bool (i.e. bool).
  RAPID_NUM,   ///< \brief RAPID num (i.e. float).
  RAPID_DNUM,  ///< \brief RAPID dnum (i.e. double).
  RAPID_STRING ///< \brief RAPID string (i.e. std::string).
};

/**
 * \brief An abstract struct, for structs representing the data of RAPID symbols.
 */
struct RAPIDSymbolDataAbstract
{
public:
  /**
   * \brief Pure virtual method for retrieving the name of the symbol's data type.
   *
   * \return std::string containing the data type name.
   */
  virtual std::string getType() const = 0;

  /**
   * \brief Pure virtual method for parsing a RAPID symbol data value string.
   *
   * \param value_string containing the string to parse.
   */
  virtual void parseString(const std::string&  value_string) = 0;

  /**
   * \brief Pure virtual method for constructing a RAPID symbol data value string.
   *
   * \return std::string containing the constructed string.
   */
  virtual std::string constructString() const = 0;
};

/**
 * \brief A template struct, for structs representing the data of RAPID atomic symbols (mapped to standard data types).
 */
template <typename T>
struct RAPIDAtomicTemplate : public RAPIDSymbolDataAbstract
{
public:
  /**
   * \brief A method for parsing a RAPID symbol data value string.
   *
   * \param value_string containing the string to parse.
   */
  void parseString(const std::string& value_string)
  {
    std::stringstream ss(value_string);
    ss >> value;
  }

  /**
   * \brief Container for the data's value.
   */
  T value;

protected:
  /**
   * \brief A constructor.
   *
   * \param value specifying the value of the data.
   */
  RAPIDAtomicTemplate(const T& value) : value(value) {}
};

/**
 * \brief Declaration of a template struct, for representing the data of RAPID atomic symbols.
 */
template<RAPIDAtomicTypes T> struct RAPIDAtomic;

/**
 * \brief A specific derived struct template, for representing the data of a RAPID bool symbol.
 */
template <>
struct RAPIDAtomic<RAPID_BOOL> : public RAPIDAtomicTemplate<bool>
{
  /**
   * \brief A constructor.
   *
   * \param value specifying the value of the data.
   */
  RAPIDAtomic(const bool value = false) : RAPIDAtomicTemplate(value) {}

  /**
   * \brief A method for retrieving the name of the symbol's data type.
   *
   * \return std::string containing the data type name.
   */
  std::string getType() const;

  /**
   * \brief A method for parsing a RAPID symbol data value string.
   *
   * \param value_string containing the string to parse.
   */
  void parseString(const std::string& value_string);

  /**
   * \brief A method for constructing a RAPID symbol data value string.
   *
   * \return std::string containing the constructed string.
   */
  std::string constructString() const;
};

/**
 * \brief A specific derived struct template, for representing the data of a RAPID num symbol.
 */
template <>
struct RAPIDAtomic<RAPID_NUM> : public RAPIDAtomicTemplate<float>
{
  /**
   * \brief A constructor.
   *
   * \param value specifying the value of the data.
   */
  RAPIDAtomic(const float value = (float) 0.0) : RAPIDAtomicTemplate(value) {}

  /**
   * \brief A method for retrieving the name of the symbol's data type.
   *
   * \return std::string containing the data type name.
   */
  std::string getType() const;

  /**
   * \brief A method for constructing a RAPID symbol data value string.
   *
   * \return std::string containing the constructed string.
   */
  std::string constructString() const;
};

/**
 * \brief A specific derived struct template, for representing the data of a RAPID dnum symbol.
 */
template <>
struct RAPIDAtomic<RAPID_DNUM> : public RAPIDAtomicTemplate<double>
{
  /**
   * \brief A constructor.
   *
   * \param value specifying the value of the data.
   */
  RAPIDAtomic(const double value = (double) 0.0) : RAPIDAtomicTemplate(value) {}

  /**
   * \brief A method for retrieving the name of the symbol's data type.
   *
   * \return std::string containing the data type name.
   */
  std::string getType() const;

  /**
   * \brief A method for constructing a RAPID symbol data value string.
   *
   * \return std::string containing the constructed string.
   */
  std::string constructString() const;
};

/**
 * \brief A specific derived struct template, for representing the data of a RAPID string symbol.
 */
template <>
struct RAPIDAtomic<RAPID_STRING> : public RAPIDAtomicTemplate<std::string>
{
  /**
   * \brief A constructor.
   *
   * \param value specifying the value of the data.
   */
  RAPIDAtomic(const std::string& value = "") : RAPIDAtomicTemplate(value) {}

  /**
   * \brief A method for retrieving the name of the symbol's data type.
   *
   * \return std::string containing the data type name.
   */
  std::string getType() const;

  /**
   * \brief A method for parsing a RAPID symbol data value string.
   *
   * \param value_string containing the string to parse.
   */
  void parseString(const std::string& value_string);

  /**
   * \brief A method for constructing a RAPID symbol data value string.
   *
   * \return std::string containing the constructed string.
   */
  std::string constructString() const;
};

/**
 * \brief Typedef for a representing RAPID bool symbols.
 */
typedef RAPIDAtomic<RAPID_BOOL> RAPIDBool;

/**
 * \brief Typedef for a representing RAPID num symbols.
 */
typedef RAPIDAtomic<RAPID_NUM> RAPIDNum;

/**
 * \brief Typedef for a representing RAPID dnum symbols.
 */
typedef RAPIDAtomic<RAPID_DNUM> RAPIDDnum;

/**
 * \brief Typedef for a representing RAPID string symbols.
 */
typedef RAPIDAtomic<RAPID_STRING> RAPIDString;

/**
 * \brief A struct, for representing the data of a RAPID record symbol.
 */
struct RAPIDRecord : public RAPIDSymbolDataAbstract
{
public:
  /**
   * \brief A constructor.
   *
   * \param record_type_name specifying the name of the RAPID record type (i.e. its name in the RAPID code).
   */
  RAPIDRecord(const std::string& record_type_name);

  /**
   * \brief A method for constructing a RAPID symbol data value string.
   *
   * \return std::string containing the constructed string.
   */
  std::string constructString() const;

  /**
   * \brief A method for parsing a RAPID symbol data value string.
   *
   * \param value_string containing the string to parse.
   */
  void parseString(const std::string& value_string);

  /**
   * \brief A method for getting the type of the RAPID record.
   *
   * \return std::string containing the type.
   */
  std::string getType() const;

  /**
   * \brief Operator for copying the RAPID record to another RAPID record.
   *
   * \param other containing the RAPID record to copy.
   *
   * \return RAPIDRecord& containing the copy.
   */
  RAPIDRecord& operator=(const RAPIDRecord& other);

protected:
  /**
   * \brief A method to remove a character from a string and count the number of times it occurred.
   *
   * \param input for the string to search.
   * \param character specifying the character to search for.
   *
   * \return unsigned int containing the number of times the character occurs.
   */
  unsigned int countCharInString(std::string input, const char character);

  /**
   * \brief A method to extract delimited substrings in a string.
   *
   * \param input containing the string with delimited substrings.
   *
   * \return std::vector<std::string> containing the extracted substrings.
   */
  std::vector<std::string> extractDelimitedSubstrings(const std::string& input);

  /**
   * \brief The record's type name.
   */
  std::string record_type_name_;

  /**
   * \brief Container for the record's components. I.e. other RAPID records or atomic RAPID data.
   */
  std::vector<RAPIDSymbolDataAbstract*> components_;
};

/**
 * \brief A struct, for representing a RAPID robjoint record.
 */
struct RobJoint : public RAPIDRecord
{
public:
  /**
   * \brief A default constructor.
   */
  RobJoint()
  :
  RAPIDRecord("robjoint")
  {
    components_.push_back(&rax_1);
    components_.push_back(&rax_2);
    components_.push_back(&rax_3);
    components_.push_back(&rax_4);
    components_.push_back(&rax_5);
    components_.push_back(&rax_6);
  }

  /**
   * \brief First robot axis.
   */
  RAPIDNum rax_1;

  /**
   * \brief Second robot axis.
   */
  RAPIDNum rax_2;

  /**
   * \brief Third robot axis.
   */
  RAPIDNum rax_3;

  /**
   * \brief Fourth robot axis.
   */
  RAPIDNum rax_4;

  /**
   * \brief Fifth robot axis.
   */
  RAPIDNum rax_5;

  /**
   * \brief Sixth robot axis.
   */
  RAPIDNum rax_6;
};

/**
 * \brief A struct, for representing a RAPID extjoint record.
 */
struct ExtJoint : public RAPIDRecord
{
public:
  /**
   * \brief A default constructor.
   */
  ExtJoint()
  :
  RAPIDRecord("extjoint")
  {
    components_.push_back(&eax_a);
    components_.push_back(&eax_b);
    components_.push_back(&eax_c);
    components_.push_back(&eax_d);
    components_.push_back(&eax_e);
    components_.push_back(&eax_f);
  }

  /**
   * \brief External axis a.
   */
  RAPIDNum eax_a;

  /**
   * \brief External axis b.
   */
  RAPIDNum eax_b;

  /**
   * \brief External axis c.
   */
  RAPIDNum eax_c;

  /**
   * \brief External axis d.
   */
  RAPIDNum eax_d;

  /**
   * \brief External axis e.
   */
  RAPIDNum eax_e;

  /**
   * \brief External axis f.
   */
  RAPIDNum eax_f;
};

/**
 * \brief A struct, for representing a RAPID jointtarget record.
 */
struct JointTarget : public RAPIDRecord
{
public:
  /**
   * \brief A default constructor.
   */
  JointTarget()
  :
  RAPIDRecord("jointtarget")
  {
    components_.push_back(&robax);
    components_.push_back(&extax);
  }

  /**
   * \brief Copy constructor.
   *
   * \param other containing the values to copy.
   */
  JointTarget(const JointTarget& other)
  :
  RAPIDRecord(other.record_type_name_)
  {
    if (this != &other)
    {
      robax = other.robax;
      extax = other.extax;
      components_.clear();
      components_.push_back(&robax);
      components_.push_back(&extax);
    }
  }

  /**
   * \brief Robot axes.
   */
  RobJoint robax;

  /**
   * \brief External axes.
   */
  ExtJoint extax;
};

/**
 * \brief A struct, for representing a RAPID pos record.
 */
struct Pos : public RAPIDRecord
{
public:
  /**
   * \brief A default constructor.
   */
  Pos()
  :
  RAPIDRecord("pos")
  {
    components_.push_back(&x);
    components_.push_back(&y);
    components_.push_back(&z);
  }

  /**
   * \brief X-value of the position.
   */
  RAPIDNum x;

  /**
   * \brief Y-value of the position.
   */
  RAPIDNum y;

  /**
   * \brief Z-value of the position.
   */
  RAPIDNum z;
};

/**
 * \brief A struct, for representing a RAPID orient record.
 */
struct Orient : public RAPIDRecord
{
public:
  /**
   * \brief A default constructor.
   */
  Orient()
  :
  RAPIDRecord("orient")
  {
    components_.push_back(&q1);
    components_.push_back(&q2);
    components_.push_back(&q3);
    components_.push_back(&q4);
  }

  /**
   * \brief Quaternion 1.
   */
  RAPIDNum q1;

  /**
   * \brief Quaternion 2.
   */
  RAPIDNum q2;

  /**
   * \brief Quaternion 3.
   */
  RAPIDNum q3;

  /**
   * \brief Quaternion 4.
   */
  RAPIDNum q4;
};

/**
 * \brief A struct, for representing a RAPID pose record.
 */
struct Pose : public RAPIDRecord
{
public:
  /**
   * \brief A default constructor.
   */
  Pose()
  :
  RAPIDRecord("pose")
  {
    components_.push_back(&pos);
    components_.push_back(&rot);
  }

  /**
   * \brief Copy constructor.
   *
   * \param other containing the values to copy.
   */
  Pose(const Pose& other)
  :
  RAPIDRecord(other.record_type_name_)
  {
    if (this != &other)
    {
      pos = other.pos;
      rot = other.rot;
      components_.clear();
      components_.push_back(&pos);
      components_.push_back(&rot);
    }
  }

  /**
   * \brief Position (x, y, z) [mm].
   */
  Pos pos;

  /**
   * \brief Rotation quaternion.
   */
  Orient rot;
};

/**
 * \brief A struct, for representing a RAPID confdata record.
 */
struct ConfData : public RAPIDRecord
{
public:
  /**
   * \brief A default constructor.
   */
  ConfData()
  :
  RAPIDRecord("confdata")
  {
    components_.push_back(&cf1);
    components_.push_back(&cf4);
    components_.push_back(&cf6);
    components_.push_back(&cfx);
  }

  /**
   * \brief Quadrent number for axis 1.
   */
  RAPIDNum cf1;

  /**
   * \brief Quadrent number for axis 4.
   */
  RAPIDNum cf4;

  /**
   * \brief Quadrent number for axis 6.
   */
  RAPIDNum cf6;

  /**
   * \brief Indicates one of eight possible robot configurations (numbered from 0-7).
   */
  RAPIDNum cfx;
};

/**
 * \brief A struct, for representing a RAPID robtarget record.
 */
struct RobTarget : public RAPIDRecord
{
public:
  /**
   * \brief A default constructor.
   */
  RobTarget()
  :
  RAPIDRecord("robtarget")
  {
    components_.push_back(&pos);
    components_.push_back(&orient);
    components_.push_back(&robconf);
    components_.push_back(&extax);
  }

  /**
   * \brief Copy constructor.
   *
   * \param other containing the values to copy.
   */
  RobTarget(const RobTarget& other)
  :
  RAPIDRecord(other.record_type_name_)
  {
    if (this != &other)
    {
      pos = other.pos;
      orient = other.orient;
      robconf = other.robconf;
      extax = other.extax;
      components_.clear();
      components_.push_back(&pos);
      components_.push_back(&orient);
      components_.push_back(&robconf);
      components_.push_back(&extax);
    }
  }

  /**
   * \brief Position for the tool center point [mm].
   */
  Pos pos;

  /**
   * \brief Orientation quaternion for the tool.
   */
  Orient orient;

  /**
   * \brief Robot axis configuration.
   */
  ConfData robconf;

  /**
   * \brief External axes.
   */
  ExtJoint extax;
};

/**
 * \brief A struct, for representing a RAPID loaddata record.
 */
struct LoadData : public RAPIDRecord
{
public:
  /**
   * \brief A default constructor.
   */
  LoadData()
  :
  RAPIDRecord("loaddata")
  {
    components_.push_back(&mass);
    components_.push_back(&cog);
    components_.push_back(&aom);
    components_.push_back(&ix);
    components_.push_back(&iy);
    components_.push_back(&iz);
  }

  /**
   * \brief Copy constructor.
   *
   * \param other containing the values to copy.
   */
  LoadData(const LoadData& other)
  :
  RAPIDRecord(other.record_type_name_)
  {
    if (this != &other)
    {
      mass = other.mass;
      cog = other.cog;
      aom = other.aom;
      ix = other.ix;
      iy = other.iy;
      iz = other.iz;
      components_.clear();
      components_.push_back(&mass);
      components_.push_back(&cog);
      components_.push_back(&aom);
      components_.push_back(&ix);
      components_.push_back(&iy);
      components_.push_back(&iz);
    }
  }

  /**
   * \brief The mass of the load [kg].
   */
  RAPIDNum mass;

  /**
   * \brief Center of gravity (x, y, z).
   */
  Pos cog;

  /**
   * \brief Axes of moment.
   */
  Orient aom;

  /**
   * \brief Inertia of the load around the x-axis [kgm^2].
   */
  RAPIDNum ix;

  /**
   * \brief Inertia of the load around the y-axis [kgm^2].
   */
  RAPIDNum iy;

  /**
   * \brief Inertia of the load around the z-axis [kgm^2].
   */
  RAPIDNum iz;
};

/**
 * \brief A struct, for representing a RAPID tooldata record.
 */
struct ToolData : public RAPIDRecord
{
public:
  /**
   * \brief A default constructor.
   */
  ToolData()
  :
  RAPIDRecord("tooldata")
  {
    components_.push_back(&robhold);
    components_.push_back(&tframe);
    components_.push_back(&tload);
  }

  /**
   * \brief Copy constructor.
   *
   * \param other containing the values to copy.
   */
  ToolData(const ToolData& other)
  :
  RAPIDRecord(other.record_type_name_)
  {
    if (this != &other)
    {
      robhold = other.robhold;
      tframe = other.tframe;
      tload = other.tload;
      components_.clear();
      components_.push_back(&robhold);
      components_.push_back(&tframe);
      components_.push_back(&tload);
    }
  }

  /**
   * \brief Defines if the robot is holding the tool or not.
   */
  RAPIDBool robhold;

  /**
   * \brief The tool's coordinate system.
   */
  Pose tframe;

  /**
   * \brief The tool's load.
   */
  LoadData tload;
};

/**
 * \brief A struct, for representing a RAPID wobjdata record.
 */
struct WObjData : public RAPIDRecord
{
public:
  /**
   * \brief A default constructor.
   */
  WObjData()
  :
  RAPIDRecord("wobjdata")
  {
    components_.push_back(&robhold);
    components_.push_back(&ufprog);
    components_.push_back(&ufmec);
    components_.push_back(&uframe);
    components_.push_back(&oframe);
  }

  /**
   * \brief Copy constructor.
   *
   * \param other containing the values to copy.
   */
  WObjData(const WObjData& other)
  :
  RAPIDRecord(other.record_type_name_)
  {
    if (this != &other)
    {
      robhold = other.robhold;
      ufprog = other.ufprog;
      ufmec = other.ufmec;
      uframe = other.uframe;
      oframe = other.oframe;
      components_.clear();
      components_.push_back(&robhold);
      components_.push_back(&ufprog);
      components_.push_back(&ufmec);
      components_.push_back(&uframe);
      components_.push_back(&oframe);
    }
  }

  /**
   * \brief Defines if the robot is holding the work object or not.
   */
  RAPIDBool robhold;

  /**
   * \brief User frame programmed.
   */
  RAPIDBool ufprog;

  /**
   * \brief User frame mechanical unit.
   */
  RAPIDString ufmec;

  /**
   * \brief User frame.
   */
  Pose uframe;

  /**
  * \brief Object frame.
  */
  Pose oframe;
};

/**
 * \brief A struct, for representing a RAPID speeddata record.
 */
struct SpeedData : public RAPIDRecord
{
public:
  /**
   * \brief A default constructor.
   */
  SpeedData()
  :
  RAPIDRecord("speeddata")
  {
    components_.push_back(&v_tcp);
    components_.push_back(&v_ori);
    components_.push_back(&v_leax);
    components_.push_back(&v_reax);
  }

  /**
   * \brief The speed [mm/s] of the tool center point (TCP).
   */
  RAPIDNum v_tcp;

  /**
   * \brief The reorientation speed [deg/s] of the tool center point (TCP).
   */
  RAPIDNum v_ori;

  /**
   * \brief The linear speed [mm/s] of external axes.
   */
  RAPIDNum v_leax;

  /**
   * \brief The rotational speed [degrees/s] of external axes.
   */
  RAPIDNum v_reax;
};

} // end namespace rws
} // end namespace abb

#endif
