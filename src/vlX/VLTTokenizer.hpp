/**************************************************************************************/
/*                                                                                    */
/*  Visualization Library                                                             */
/*  http://visualizationlibrary.org                                                   */
/*                                                                                    */
/*  Copyright (c) 2005-2020, Michele Bosi                                             */
/*  All rights reserved.                                                              */
/*                                                                                    */
/*  Redistribution and use in source and binary forms, with or without modification,  */
/*  are permitted provided that the following conditions are met:                     */
/*                                                                                    */
/*  - Redistributions of source code must retain the above copyright notice, this     */
/*  list of conditions and the following disclaimer.                                  */
/*                                                                                    */
/*  - Redistributions in binary form must reproduce the above copyright notice, this  */
/*  list of conditions and the following disclaimer in the documentation and/or       */
/*  other materials provided with the distribution.                                   */
/*                                                                                    */
/*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND   */
/*  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED     */
/*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE            */
/*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR  */
/*  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    */
/*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      */
/*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON    */
/*  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT           */
/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS     */
/*  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                      */
/*                                                                                    */
/**************************************************************************************/

#ifndef VLTTokenizer_INCLUDE_ONCE
#define VLTTokenizer_INCLUDE_ONCE

#include <vlX/link_config.hpp>
#include <vlCore/BufferedStream.hpp>

namespace vlX
{
  /** A token of information as defined by VLT specification. */
  class VLTToken
  {
  public:
    typedef enum
    {
      TOKEN_ERROR,
      TOKEN_EOF,

      LeftRoundBracket,   //  (
      RightRoundBracket,  //  )
      LeftSquareBracket,  //  [
      RightSquareBracket, //  ]
      LeftCurlyBracket,   //  {
      RightCurlyBracket,  //  }
      LeftFancyBracket,   //  {<
      RightFancyBracket,  //  >}
      Equals,             //  =
      String,             //  "nel mezzo del cammin di nostra vita\n"
      ID,                 //  #unique_id123
      Identifier,         //  Identifier_123
      Boolean,            //  true | false
      Integer,            //  +123
      real,               //  +123.456e+10
      TagHeader,          //  <TagHeader>
      RawtextBlock,         // {< blabla >}

    } EType;

    VLTToken(): mType(TOKEN_ERROR) {}

    std::string mString;
    EType mType;
  };
  //-----------------------------------------------------------------------------
  /** Tokenizer used to parse VLT files. */
  class VLTTokenizer: public vl::BufferedStream<char, 128*1024>
  {
    VL_INSTRUMENT_CLASS(vl::VLTTokenizer, VL_GROUP(vl::BufferedStream<char, 128*1024>))

  public:
    VLTTokenizer(): mLineNumber(1), mRawtextBlock(false) {}

    VLX_EXPORT bool getToken(VLTToken& token);

    VLX_EXPORT bool getRawtextBlock(VLTToken& token);

    int lineNumber() const { return mLineNumber; }

  private:
    int mLineNumber;
    bool mRawtextBlock;
  };
}

#endif
