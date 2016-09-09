#ifndef _SOLVER_HPP_
#define _SOLVER_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>
#include <vector>

#include "solvlet.hpp"
#include "sequence.hpp"

namespace SPEL
{  
  /// <summary>
  /// Base Solver class.
  /// </summary>
  class Solver
  {
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="Solver"/> class.
    /// </summary>
    Solver(void) noexcept;
    /// <summary>
    /// Finalizes an instance of the <see cref="Solver"/> class.
    /// </summary>
    virtual ~Solver(void) noexcept;    
    /// <summary>Solves the specified <see cref="Sequence"/>.</summary>
    /// <param name="sequence">The <see cref="Sequence"/>.</param>
    /// <returns>Array of <see cref="Solvlet"/>.</returns>
    virtual std::vector<Solvlet> solve(Sequence& sequence) = 0;    
    /// <summary>Solves the specified sequence.</summary>
    /// <param name="sequence">The <see cref="Sequence"/>.</param>
    /// <param name="params">The parameters.</param>
    /// <returns>Array of <see cref="Solvlet"/>.</returns>
    virtual std::vector<Solvlet> solve(Sequence& sequence,
      std::map<std::string, float> params) = 0;
    /// <summary>Gets the solver name.</summary>
    /// <returns>The solver name.</returns>
    std::string getName(void) const noexcept;
    /// <summary>Gets the solver identifier.</summary>
    /// <returns>The solver identifier.</returns>  
    int getId(void) const noexcept;
  protected:    
    /// <summary>The identifier.</summary>
    int id;    
    /// <summary>The name.</summary>
    std::string name;
  };

}

#endif  // _SOLVER_HPP_