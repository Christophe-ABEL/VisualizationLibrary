/**************************************************************************************/
/*                                                                                    */
/*  Visualization Library                                                             */
/*  http://www.visualizationlibrary.org                                               */
/*                                                                                    */
/*  Copyright (c) 2005-2010, Michele Bosi                                             */
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

#ifndef TriangleAccessor_INCLUDE_ONCE
#define TriangleAccessor_INCLUDE_ONCE

#include <vlGraphics/Array.hpp>
#include <vlCore/vlnamespace.hpp>
#include <vlCore/Say.hpp>

namespace vl
{
//-----------------------------------------------------------------------------
// TriangleAccessorAbstract
//-----------------------------------------------------------------------------
  /** For internal use only. See vl::TriangleAccessor instead. */
  class TriangleAccessorAbstract: public Object
  {
    VL_INSTRUMENT_ABSTRACT_CLASS(vl::TriangleAccessorAbstract, Object)

  public:
    virtual int a(unsigned int idx) = 0;
    virtual int b(unsigned int idx) = 0;
    virtual int c(unsigned int idx) = 0;
  };
//-----------------------------------------------------------------------------
// TriangleAccessorIndexed
//-----------------------------------------------------------------------------
  /** For internal use only. See vl::TriangleAccessor instead. */
  template<class TArray>
  class TriangleAccessorIndexed: public TriangleAccessorAbstract
  {
    VL_INSTRUMENT_CLASS(vl::TriangleAccessorIndexed<TArray>, TriangleAccessorAbstract)

  public:

    TriangleAccessorIndexed()
    {
      VL_DEBUG_SET_OBJECT_NAME()
      mCurrentIndex = 0;
      mEnd    = 0;
      mA = mB = mC = -1;
      mEven = true;
      mIndex0 = 0;
      mArray            = NULL;
      mPrimRestartIndex = (unsigned int)-1;
      mPrimRestartOn    = false;
      mBaseVertex       = 0;
      mPrimType         = PT_UNKNOWN;
      mPrimRestartNumber = 0;
    }

    TriangleAccessorIndexed(const TArray* idx_array, EPrimitiveType prim_type, int base_vert, bool prim_restart_on, unsigned int prim_restart_idx)
    {
      VL_DEBUG_SET_OBJECT_NAME()
      mCurrentIndex = 0;
      mEnd    = 0;
      mA = mB = mC = -1;
      mEven = true;
      mIndex0 = 0;
      mArray            = idx_array;
      mPrimRestartIndex = prim_restart_idx;
      mPrimRestartOn    = prim_restart_on;
      mBaseVertex       = base_vert;
      mPrimType         = prim_type;
      if(mPrimRestartOn)
      {
          for(unsigned int i = 0; i < mArray->size(); ++i)
              if(mArray->at(i) == mPrimRestartIndex)
                  mPrimRestartNumber++;
      }
    }

    //bool hasNext() const { return mCurrentIndex != mEnd; }

    virtual int a(unsigned int idx) { get(idx); return mA; }
    virtual int b(unsigned int idx) { get(idx); return mB; }
    virtual int c(unsigned int idx) { get(idx); return mC; }

    void get(unsigned int idx)
    {
        if(idx > static_cast< unsigned int >(mEnd))
            return;

        switch(mPrimType)
        {
        case PT_TRIANGLES:
          //mCurrentIndex += 3;
          // check for the end
          if ( idx * 3 + 3 > static_cast< unsigned int >(mEnd) )
            //mCurrentIndex = mEnd;
              break;
          /*else
          if ( isPrimRestart(mCurrentIndex) )
          {
            mCurrentIndex += 1;
            mA = mArray->at(idx * 3 + 0);
            mB = mArray->at(idx * 3 + 1);
            mC = mArray->at(idx * 3 + 2);
          }
          else
          {*/
            mA = mArray->at(idx * 3 + 0);
            mB = mArray->at(idx * 3 + 1);
            mC = mArray->at(idx * 3 + 2);
          //}
          break;

        case PT_QUAD_STRIP:
        case PT_TRIANGLE_STRIP:
          //mCurrentIndex += 1;
          if ( idx * 3 + 2 >= static_cast< unsigned int >(mEnd) )
            idx = mEnd;
          /*else
          if ( isPrimRestart(mCurrentIndex + 2) )
          {
            mCurrentIndex += 3;
            mEven = true;
            mA = mArray->at(mCurrentIndex + 0);
            mB = mArray->at(mCurrentIndex + 1);
            mC = mArray->at(mCurrentIndex + 2);
          }
          else
          {*/
            mEven = !mEven;
            if (mEven)
            {
              mA = mArray->at(idx * 3 + 0);
              mB = mArray->at(idx * 3 + 1);
              mC = mArray->at(idx * 3 + 2);
            }
            else
            {
              mA = mArray->at(idx * 3 + 0);
              mB = mArray->at(idx * 3 + 2);
              mC = mArray->at(idx * 3 + 1);
            }
          //}
          break;

        case PT_TRIANGLE_FAN:
        case PT_POLYGON:
          //mCurrentIndex += 1;
          if ( idx + 1 >= static_cast< unsigned int >(mEnd) )
          {
            //mCurrentIndex = mEnd;
              idx = mEnd;
          }
          /*else
          if ( isPrimRestart(mCurrentIndex + 1) )
          {
            mIndex0 = mCurrentIndex + 2;
            mCurrentIndex = mIndex0 + 1;
            mA = mArray->at(mIndex0);
            mB = mArray->at(mCurrentIndex + 0);
            mC = mArray->at(mCurrentIndex + 1);
          }
          else
          {*/
            mA = mArray->at(mIndex0);
            mB = mArray->at(idx + 0);
            mC = mArray->at(idx + 1);
          //}
          break;

        case PT_QUADS:
          //mCurrentIndex += 2;
          if ( mCurrentIndex + 2 >= mEnd )
          {
            idx = mEnd;
          }
          /*else
          if ( isPrimRestart(mCurrentIndex) )
          {
            mCurrentIndex += 1;
            mEven = true;
            mA = mArray->at(mCurrentIndex+0);
            mB = mArray->at(mCurrentIndex+1);
            mC = mArray->at(mCurrentIndex+2);
          }
          else
          {*/
            mEven = !mEven;
            if ( mEven )
            {
              mA = mArray->at(idx + 0);
              mB = mArray->at(idx + 1);
              mC = mArray->at(idx + 2);
            }
            else
            {
              mA = mArray->at(idx + 0);
              mB = mArray->at(idx + 1);
              mC = mArray->at(idx - 2);
            }
         // }
          break;

        default:
          VL_TRAP();
          break;
        }

        // if we are not at the end then add base vertices
        if (idx != static_cast< unsigned int >(mEnd) )
        {
          mA += mBaseVertex;
          mB += mBaseVertex;
          mC += mBaseVertex;
          return;
        }
        else
        {
          mA = mB = mC = -1;
          return;
        }
    }

    void initialize(int start=0, int end=-1)
    {
      VL_CHECK( start >= 0 )
      VL_CHECK( end <= (int)mArray->size() )

      if (end == -1)
        end = (int)mArray->size();

      mCurrentIndex = end; // end
      mA = mB = mC = -1;
      mEven = true;
      mIndex0 = start;
      mEnd    = end;
      /*if (mArray->size())
      {
        switch(mPrimType)
        {
        case PT_TRIANGLES:
          mCurrentIndex = start;
          mA = mArray->at(start+0);
          mB = mArray->at(start+1);
          mC = mArray->at(start+2);
          break;
        case PT_TRIANGLE_STRIP:
          mCurrentIndex = start;
          mA = mArray->at(start+0);
          mB = mArray->at(start+1);
          mC = mArray->at(start+2);
          break;
        case PT_TRIANGLE_FAN:
        case PT_POLYGON:
          mCurrentIndex = start + 1;
          mA = mArray->at(start+0);
          mB = mArray->at(start+1);
          mC = mArray->at(start+2);
          break;
        case PT_QUADS:
          mCurrentIndex = start;
          mA = mArray->at(start+0);
          mB = mArray->at(start+1);
          mC = mArray->at(start+2);
          break;
        case PT_QUAD_STRIP:
          mCurrentIndex = start;
          mA = mArray->at(start+0);
          mB = mArray->at(start+1);
          mC = mArray->at(start+2);
          break;
        default:
          break;
        }
      }

      // if we are not at the end then add base vertices
      if ( mCurrentIndex != mEnd )
      {
        mA += mBaseVertex;
        mB += mBaseVertex;
        mC += mBaseVertex;
      }
      else
      {
        mA = mB = mC = -1;
      }*/
    }

    /*bool next()
    {
      // reached the end
      if ( mCurrentIndex == mEnd )
        return false;

      switch(mPrimType)
      {

      case PT_TRIANGLES:
        mCurrentIndex += 3;
        // check for the end
        if ( mCurrentIndex >= mEnd )
          mCurrentIndex = mEnd;
        else
        if ( isPrimRestart(mCurrentIndex) )
        {
          mCurrentIndex += 1;
          mA = mArray->at(mCurrentIndex + 0);
          mB = mArray->at(mCurrentIndex + 1);
          mC = mArray->at(mCurrentIndex + 2);
        }
        else
        {
          mA = mArray->at(mCurrentIndex + 0);
          mB = mArray->at(mCurrentIndex + 1);
          mC = mArray->at(mCurrentIndex + 2);
        }
        break;

      case PT_QUAD_STRIP:
      case PT_TRIANGLE_STRIP:
        mCurrentIndex += 1;
        if ( mCurrentIndex + 2 >= mEnd )
          mCurrentIndex = mEnd;
        else
        if ( isPrimRestart(mCurrentIndex + 2) )
        {
          mCurrentIndex += 3;
          mEven = true;
          mA = mArray->at(mCurrentIndex + 0);
          mB = mArray->at(mCurrentIndex + 1);
          mC = mArray->at(mCurrentIndex + 2);
        }
        else
        {
          mEven = !mEven;
          if (mEven)
          {
            mA = mArray->at(mCurrentIndex + 0);
            mB = mArray->at(mCurrentIndex + 1);
            mC = mArray->at(mCurrentIndex + 2);
          }
          else
          {
            mA = mArray->at(mCurrentIndex + 0);
            mB = mArray->at(mCurrentIndex + 2);
            mC = mArray->at(mCurrentIndex + 1);
          }
        }
        break;

      case PT_TRIANGLE_FAN:
      case PT_POLYGON:
        mCurrentIndex += 1;
        if ( mCurrentIndex + 1 >= mEnd )
        {
          mCurrentIndex = mEnd;
        }
        else
        if ( isPrimRestart(mCurrentIndex + 1) )
        {
          mIndex0 = mCurrentIndex + 2;
          mCurrentIndex = mIndex0 + 1;
          mA = mArray->at(mIndex0);
          mB = mArray->at(mCurrentIndex + 0);
          mC = mArray->at(mCurrentIndex + 1);
        }
        else
        {
          mA = mArray->at(mIndex0);
          mB = mArray->at(mCurrentIndex + 0);
          mC = mArray->at(mCurrentIndex + 1);
        }
        break;

      case PT_QUADS:
        mCurrentIndex += 2;
        if ( mCurrentIndex >= mEnd )
        {
          mCurrentIndex = mEnd;
        }
        else
        if ( isPrimRestart(mCurrentIndex) )
        {
          mCurrentIndex += 1;
          mEven = true;
          mA = mArray->at(mCurrentIndex+0);
          mB = mArray->at(mCurrentIndex+1);
          mC = mArray->at(mCurrentIndex+2);
        }
        else
        {
          mEven = !mEven;
          if ( mEven )
          {
            mA = mArray->at(mCurrentIndex+0);
            mB = mArray->at(mCurrentIndex+1);
            mC = mArray->at(mCurrentIndex+2);
          }
          else
          {
            mA = mArray->at(mCurrentIndex+0);
            mB = mArray->at(mCurrentIndex+1);
            mC = mArray->at(mCurrentIndex-2);
          }
        }
        break;

      default:
        VL_TRAP();
        break;
      }

      // if we are not at the end then add base vertices
      if (mCurrentIndex != mEnd)
      {
        mA += mBaseVertex;
        mB += mBaseVertex;
        mC += mBaseVertex;
        return true;
      }
      else
      {
        mA = mB = mC = -1;
        return false;
      }
    }*/

    void setBaseVertex(int base_vert) { mBaseVertex = base_vert; }
    int baseVertex() const { return mBaseVertex; }

  private:
    bool isPrimRestart(int i) const { return mPrimRestartOn && mArray->at(i) == mPrimRestartIndex; }

  private:
    const TArray* mArray;
    EPrimitiveType mPrimType;
    int  mA, mB, mC;
    int  mCurrentIndex;
    int  mIndex0;
    int  mEnd;
    int  mBaseVertex;
    unsigned int mPrimRestartIndex;
    bool mPrimRestartOn;
    bool mEven;
    unsigned int mPrimRestartNumber;
  };
//-----------------------------------------------------------------------------
// TriangleAccessorDirect
//-----------------------------------------------------------------------------
  /** For internal use only. See vl::TriangleAccessor instead. */
  class TriangleAccessorDirect: public TriangleAccessorAbstract
  {
    VL_INSTRUMENT_CLASS(vl::TriangleAccessorDirect, TriangleAccessorAbstract)

  public:
    TriangleAccessorDirect(EPrimitiveType prim_type=PT_UNKNOWN)
    {
      VL_DEBUG_SET_OBJECT_NAME()
      mCurrentIndex = mStart = mEnd = 0;
      mA = mB = mC = -1;
      mPrimType = prim_type;
      mEven = true;
    }

    bool hasNext() const { return mCurrentIndex != mEnd; }

    virtual int a(unsigned int idx) { get(idx); return mA; }
    virtual int b(unsigned int idx) { get(idx); return mB; }
    virtual int c(unsigned int idx) { get(idx); return mC; }

    void get(unsigned int idx)
    {
        // reached the end
        if ( idx * 3 >= static_cast< unsigned int >(mEnd) )
          return;

        switch(mPrimType)
        {
        case PT_TRIANGLES:
          // VL_CHECK( (end - start) % 3 == 0 ) /* primitive restart might screw up this */
          //mCurrentIndex = start;
          mA = idx * 3 + 0;
          mB = idx * 3 + 1;
          mC = idx * 3 + 2;
          break;

        case PT_QUAD_STRIP:
            break;
        case PT_TRIANGLE_STRIP:
          //mCurrentIndex += 1;

          if ( idx + 1 >= static_cast< unsigned int >(mEnd) )
            break;
          else
          {
            mEven = !mEven;
            if (mEven)
            {
              mA = idx + 0;
              mB = idx + 1;
              mC = idx + 2;
            }
            else
            {
              mA = idx + 0;
              mB = idx + 2;
              mC = idx + 1;
            }
          }
          break;

        case PT_TRIANGLE_FAN:
        case PT_POLYGON:
          //mCurrentIndex += 1;
          if ( idx + 2 >= static_cast< unsigned int >(mEnd) )
          {
            break;
          }
          else
          {
            mA = mStart;
            mB = idx + 1;
            mC = idx + 2;
          }
          break;

        case PT_QUADS:
          mCurrentIndex += 2;
          if ( idx + 2 >= static_cast< unsigned int >(mEnd) )
          {
            //mCurrentIndex = mEnd;
            break;
          }
          else
          {
            mEven = !mEven;
            if ( mEven )
            {
              mA = idx + 0;
              mB = idx + 1;
              mC = idx + 2;
            }
            else
            {
              mA = idx + 0;
              mB = idx + 1;
              mC = idx - 2;
            }
          }
          break;

        default:
          VL_TRAP();
          break;
        }
    }

    void initialize(int start, int end)
    {
      VL_CHECK(end >= start)
      mStart = start;
      mCurrentIndex = mEnd = end;
      mA = mB = mC = -1;
      mEven = true;
      /*switch(mPrimType)
      {
      case PT_TRIANGLES:
        mCurrentIndex = start;
        mA = start + 0;
        mB = start + 1;
        mC = start + 2;
        break;
      case PT_TRIANGLE_STRIP:
        mCurrentIndex = start;
        mA = start + 0;
        mB = start + 1;
        mC = start + 2;
        break;
      case PT_TRIANGLE_FAN:
      case PT_POLYGON:
        mCurrentIndex = start + 1;
        mA = start + 0;
        mB = start + 1;
        mC = start + 2;
        break;
      case PT_QUADS:
        mCurrentIndex = start;
        mA = start + 0;
        mB = start + 1;
        mC = start + 2;
        break;
      case PT_QUAD_STRIP:
        mCurrentIndex = start;
        mA = start + 0;
        mB = start + 1;
        mC = start + 2;
        break;
      default:
        break;
      }*/
    }

    /*bool next()
    {
      // reached the end
      if ( mCurrentIndex == mEnd )
        return false;

      switch(mPrimType)
      {

      case PT_TRIANGLES:
        mCurrentIndex += 3;
        // check for the end
        if ( mCurrentIndex >= mEnd )
          mCurrentIndex = mEnd;
        else
        {
          mA = mCurrentIndex + 0;
          mB = mCurrentIndex + 1;
          mC = mCurrentIndex + 2;
        }
        break;

      case PT_QUAD_STRIP:
      case PT_TRIANGLE_STRIP:
        mCurrentIndex += 1;
        if ( mCurrentIndex + 2 >= mEnd )
          mCurrentIndex = mEnd;
        else
        {
          mEven = !mEven;
          if (mEven)
          {
            mA = mCurrentIndex + 0;
            mB = mCurrentIndex + 1;
            mC = mCurrentIndex + 2;
          }
          else
          {
            mA = mCurrentIndex + 0;
            mB = mCurrentIndex + 2;
            mC = mCurrentIndex + 1;
          }
        }
        break;

      case PT_TRIANGLE_FAN:
      case PT_POLYGON:
        mCurrentIndex += 1;
        if ( mCurrentIndex + 1 >= mEnd )
        {
          mCurrentIndex = mEnd;
        }
        else
        {
          mA = mStart;
          mB = mCurrentIndex+0;
          mC = mCurrentIndex+1;
        }
        break;

      case PT_QUADS:
        mCurrentIndex += 2;
        if ( mCurrentIndex >= mEnd )
        {
          mCurrentIndex = mEnd;
        }
        else
        {
          mEven = !mEven;
          if ( mEven )
          {
            mA = mCurrentIndex+0;
            mB = mCurrentIndex+1;
            mC = mCurrentIndex+2;
          }
          else
          {
            mA = mCurrentIndex+0;
            mB = mCurrentIndex+1;
            mC = mCurrentIndex-2;
          }
        }
        break;

      default:
        VL_TRAP();
        break;
      }

      // if we are not at the end then add base vertices
      if (mCurrentIndex == mEnd)
      {
        mA = mB = mC = -1;
        return false;
      }
      else
        return true;
    }*/

  private:
    EPrimitiveType mPrimType;
    int  mA, mB, mC;
    int  mCurrentIndex;
    int  mStart;
    int  mEnd;
    bool mEven;
  };
//-----------------------------------------------------------------------------
// TriangleAccessorMulti
//-----------------------------------------------------------------------------
  /** For internal use only. See vl::TriangleAccessor instead. */
  template<class TArray>
  class TriangleAccessorMulti: public TriangleAccessorIndexed<TArray>
  {
    VL_INSTRUMENT_CLASS(vl::TriangleAccessorMulti<class TArray>, TriangleAccessorIndexed<TArray>)

  public:
    TriangleAccessorMulti( const std::vector<GLint>* p_base_vertices, const std::vector<GLsizei>* p_count_vector, const TArray* idx_array, EPrimitiveType prim_type, bool prim_restart_on, int prim_restart_idx)
    :TriangleAccessorIndexed<TArray>( idx_array, prim_type, 0, prim_restart_on, prim_restart_idx)
    {
      VL_DEBUG_SET_OBJECT_NAME()
      mpBaseVertices  = p_base_vertices;
      mpCountVector   = p_count_vector;
      mStart   = 0;
      mCurPrim = 0;
    }

    void initialize()
    {
      /*VL_CHECK( mpBaseVertices->size() == mpCountVector->size() )
      if ( (*mpBaseVertices).size() )
        TriangleAccessorIndexed<TArray>::setBaseVertex( (*mpBaseVertices)[mCurPrim] );
      int end = mStart + (*mpCountVector)[mCurPrim];
      TriangleAccessorIndexed<TArray>::initialize( mStart, end );
      // abort if could not initialize (primitive not supported)
      if ( !TriangleAccessorIndexed<TArray>::hasNext() )
        mCurPrim = (int)(*mpCountVector).size()-1;*/
    }

    bool next() 
    { 
     /* if ( TriangleAccessorIndexed<TArray>::next() )
        return true;
      else
      if ( mCurPrim < (int)(*mpCountVector).size()-1 )
      {
        mStart += (*mpCountVector)[mCurPrim];
        mCurPrim++;
        initialize();
        return true;
      }
      else*/
        return false;
    }

    bool hasNext() const
    { 
      /*if ( !TriangleAccessorIndexed<TArray>::hasNext() && mCurPrim == (int)(*mpCountVector).size()-1 )
        return false;
      else*/
        return true;
    }

  protected:
    const std::vector<GLint>* mpBaseVertices;
    const std::vector<GLsizei>* mpCountVector;
    int mCurPrim;
    int mStart;
  };
//-----------------------------------------------------------------------------
// TriangleAccessor
//-----------------------------------------------------------------------------
  /** Iterator used to extract the indices of every single triangle of a DrawCall 
    * regardless of the primitive type.
    * \sa DrawCall::triangles() */
  class TriangleAccessor
  {
  public:
    TriangleAccessor(TriangleAccessorAbstract* it): mAccessor(it) { }

    /** Requires the next triangle. Returns \p false the iterator reached the end of the triangle list. */
    //bool next() { return mIterator->next(); }

    //bool operator++() { return next(); }

    /** Returns false if the iterator has reached the end of the triangle list. In this case a(), b() and c() return -1. */
    //bool hasNext() { return mIterator->hasNext(); }

    /** First index of the triangle. */
    int a(unsigned int idx) { return mAccessor->a(idx); }

    /** First index of the triangle. */
    int b(unsigned int idx) { return mAccessor->b(idx); }

    /** First index of the triangle. */
    int c(unsigned int idx) { return mAccessor->c(idx); }

  protected:
    ref<TriangleAccessorAbstract> mAccessor;
  };
//-----------------------------------------------------------------------------
}

#endif
