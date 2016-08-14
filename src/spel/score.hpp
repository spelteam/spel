#ifndef _SCORE_HPP_
#define _SCORE_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>

namespace SPEL
{
  /// <summary>
  /// Used to evaluate accuracy of a detection
  /// </summary>
  class Score
  {
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="Score"/> class.
    /// </summary>
    Score(void) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="Score"/> class.
    /// </summary>
    /// <param name="score">The score.</param>
    /// <param name="name">The detector name.</param>
    Score(const float score, const std::string &name) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="Score"/> class.
    /// </summary>
    /// <param name="score">The score.</param>
    /// <param name="name">The detector name.</param>
    /// <param name="coeff">The coefficient.</param>
    Score(const float score, const std::string &name, const float coeff) 
      noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="Score"/> class.
    /// </summary>
    /// <param name="score">The score.</param>
    /// <param name="name">The detector name.</param>
    /// <param name="coeff">The coefficient.</param>
    /// <param name="isWeak">The flag of weak score.</param>
    Score(const float score, const std::string &name, const float coeff, 
      const bool isWeak) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="Score"/> class.
    /// Copy constructor.
    /// </summary>
    /// <param name="score">The score.</param>
    Score(const Score& score) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="Score"/> class.
    /// Move constructor.
    /// </summary>
    /// <param name="score">The score.</param>
    Score(Score&& score) noexcept;
    /// <summary>
    /// Finalizes an instance of the <see cref="Score"/> class.
    /// </summary>
    virtual ~Score(void) noexcept;
    /// <summary>Assignment operator.</summary>
    /// <param name="score">The score.</param>
    /// <returns>New score.</returns>
    Score & operator=(const Score &score) noexcept;
    /// <summary>Comparison operator.</summary>
    /// <param name="score">The score.</param>
    /// <returns>The result of the comparison.</returns>
    bool operator<(const Score &score) const noexcept;
    /// <summary>Comparison operator.</summary>
    /// <param name="score">The score.</param>
    /// <returns>The result of the comparison.</returns>
    bool operator>(const Score &score) const noexcept;
    /// <summary>Comparison operator.</summary>
    /// <param name="score">The score.</param>
    /// <returns>The result of the comparison.</returns>
    bool operator==(const Score &score) const noexcept;
    /// <summary>Comparison operator.</summary>
    /// <param name="score">The score.</param>
    /// <returns>The result of the comparison.</returns>
    bool operator!=(const Score &score) const noexcept;
    /// <summary>Move operator.</summary>
    /// <param name="score">The score.</param>
    /// <returns>New score.</returns>
    Score& operator=(Score&& score) noexcept;
    /// <summary>Gets the score.</summary>
    /// <returns>The score.</returns>
    float getScore(void) const noexcept;
    /// <summary>Sets the score.</summary>
    /// <param name="score">The score.</param>
    void setScore(const float score) noexcept;
    /// <summary>Gets the name of the detector.</summary>
    /// <returns>The name of the detector.</returns>
    std::string getDetName(void) const noexcept;
    /// <summary>Sets the name of the detector.</summary>
    /// <param name="detName">Name of the detector.</param>
    void setDetName(const std::string &detName) noexcept;
    /// <summary>Gets the coefficient.</summary>
    /// <returns>The coefficient.</returns>
    float getCoeff(void) const noexcept;
    /// <summary>Sets the coefficient.</summary>
    /// <param name="coeff">The coefficient.</param>
    void setCoeff(const float coeff) noexcept;
    /// <summary>Gets the flag of the weak score.</summary>
    /// <returns>The flag of the weak score.</returns>
    bool getIsWeak(void) const noexcept;
    /// <summary>Sets the flag of the weak score.</summary>
    /// <param name="isWeak">The flag of the weak score.</param>
    void setIsWeak(const bool isWeak) noexcept;

  private:    
    /// <summary>The score.</summary>
    float m_score;    
    /// <summary>The detector name.</summary>
    std::string m_detName;    
    /// <summary>The coefficient.</summary>
    float m_coeff;    
    /// <summary>The flag of the weak score.</summary>
    bool m_isWeak;
  };
}

#endif  // _SCORE_HPP_
