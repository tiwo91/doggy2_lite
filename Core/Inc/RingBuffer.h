/**
* \file RingBuffer.h
* \date 2014.09.30
*
* \brief Declaration of class RingBuffer
*
* \author Max Risler
* \author Dennis Schüthe <d.schuethe@uni-bremen.de>
*
* \note Ring Buffer Class is originally from
* <a href="https://github.com/bhuman/BHumanCodeRelease/tree/master/Src/Tools">bHuman</a>
* and adapted here for specific needs by the second author
*/

#pragma once

#include <string.h>

/**
* @class RingBuffer
* \brief template class for cyclic buffering of the last n values of Type V
*/
template <class V, int n> class RingBuffer
{
public:
  /** Constructor */
  RingBuffer() {init();}

  /**
  * initializes the Ringbuffer
  */
  inline void init()
  {
	  current = n - 1;
	  numberOfEntries = 0;
	  for (int i=0;i<n;i++) {
		  buffer[i] = V();
	  }
  }

  /**
   * adds an entry to the buffer
   * \param v value to be added
   */
  inline void add(const V& v)
  {
    add();
    buffer[current] = v;
  }

  /**
   * adds an entry to the buffer.
   * The new head is not initialized, but can be changed afterwards.
   */
  inline void add()
  {
    current++;
    current %= n;
    if(++numberOfEntries >= n) numberOfEntries = n;
  }

  /**
  * removes the first added entry to the buffer
  */
  inline void removeFirst()
  {
    --numberOfEntries;
  }

  /**
   * \brief removes and returns the first added entry to the buffer
   * \return first element of the buffer
   */
  inline V& rmGetFirst()
  {
    removeFirst();
    return buffer[(n + current - numberOfEntries) % n];
  }

  /**
   * \brief removes the last added entry from the buffer
   */
  inline void rmLast()
  {
    if (--current <= -1) current = 0;
    if (--numberOfEntries <= -1) numberOfEntries = 0;
  }

  /**
  * \brief Sets the buffer to 0 elements, like clearing the buffer
  */
  inline void clearBuffer(void)
  {
    numberOfEntries = 0;
  }

  /**
  * returns an entry
  * \param i index of entry counting from last added (last=0,...)
  * \return a reference to the buffer entry
  */
  inline V& getEntry(int i)
  {
    return buffer[(n + current - i) % n];
  }

  /**
  * returns an const entry
  * \param i index of entry counting from last added (last=0,...)
  * \return a reference to the buffer entry
  */
  inline const V& getEntry(int i) const
  {
    return buffer[(n + current - i) % n];
  }

  /**
  * returns an entry
  * \param i index of entry counting from last added (last=0,...)
  * \return a reference to the buffer entry
  */
  inline V& operator[](int i)
  {
    return buffer[(n + current - i) % n];
  }

  /**
  * returns a constant entry.
  * \param i index of entry counting from last added (last=0,...)
  * \return a reference to the buffer entry
  */
  inline const V& operator[](int i) const
  {
    return buffer[(n + current - i) % n];
  }

  /**
   * \brief Returns the i oldest entries of elements
   * \param values - Pointer to array of i elements
   * \param i - number of elements to be returned
   * \return none
   * \note i should be less or equal to \ref getNumberOfEntries(),
   * if greater than only number of entries are copied
   */
  inline void getCopyFirstNValues(V* values, int i) const
  {
    if ( i>numberOfEntries )
    {
      i = numberOfEntries;
    }
    for(int j=numberOfEntries-1; j>=numberOfEntries-i; j--)
    {
      *values++ = getEntry(j);
    }
  }

  /**
   * \brief Returns i elements starting with most actual one
   * \param values - Pointer to array of i elements
   * \param i - number of elements to be returned
   * \return none
   * \note i should be less or equal to \ref getNumberOfEntries(),
   * if i is greater, then all entries are copied
   */
  inline void getCopyLastNValues(V* values, int i) const
  {
    if ( i>numberOfEntries )
    {
      i = 1;
    }

    for(int j=numberOfEntries-i; j>=0; j--)
    {
      values[--i] = getEntry(j);
    }
  }

  /**
   * \brief Returns i oldest elements and removes them
   * \param values - Pointer to array of i elements
   * \param i - number of elements to be returned
   * \return none
   * \note i should be less or equal to \ref getNumberOfEntries(),
   * if i is greater, then all entries are copied
   */
  inline void getCopyAndRmFirstNValues(V* values, int i)
  {
    getCopyFirstNValues(values, i);
    clearBuffer();
  }

  /** Returns the number of elements that are currently in the ring buffer
  * \return The number
  */
  inline int getNumberOfEntries() const
  {
    return numberOfEntries;
  }

  /**
  * Returns the maximum entry count.
  * \return The maximum entry count.
  */
  inline int getMaxEntries() const
  {
    return n;
  }

  /**
  * Determines whether maximum entry count equals actual number of entries.
  * @return true iff getMaxEntries == getNumberOfEntries.
  */
  inline bool isFilled() const
  {
    return getMaxEntries() == getNumberOfEntries();
  }

  /**
  * Determines whether the buffer is empty.
  * \return True, if the number of entries is 0.
  */
  inline bool isEmpty() const
  {
    return !numberOfEntries;
  }

private:
  int current;
  int numberOfEntries;
  V buffer[n];
};
