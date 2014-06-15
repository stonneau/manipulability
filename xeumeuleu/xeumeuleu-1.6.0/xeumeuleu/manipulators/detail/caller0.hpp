/*
 *   Copyright (c) 2006, Mathieu Champlon
 *   All rights reserved.
 *
 *   Redistribution  and use  in source  and binary  forms, with  or without
 *   modification, are permitted provided  that the following conditions are
 *   met :
 *
 *   . Redistributions  of  source  code  must  retain  the  above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   . Redistributions in  binary form  must reproduce  the above  copyright
 *     notice, this list of conditions  and the following disclaimer in  the
 *     documentation and/or other materials provided with the distribution.
 *
 *   . Neither  the name  of  the  copyright  holders  nor the names  of the
 *     contributors may be used to endorse  or promote products derived from
 *     this software without specific prior written permission.
 *
 *   THIS SOFTWARE  IS  PROVIDED  BY THE  COPYRIGHT HOLDERS  AND CONTRIBUTORS
 *   ``AS IS''  AND ANY  EXPRESS OR  IMPLIED WARRANTIES,  INCLUDING,  BUT NOT
 *   LIMITED TO, THE IMPLIED  WARRANTIES  OF MERCHANTABILITY AND  FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE THE COPYRIGHT
 *   OWNERS OR CONTRIBUTORS  BE LIABLE FOR ANY  DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL,  EXEMPLARY,  OR  CONSEQUENTIAL   DAMAGES  (INCLUDING,  BUT  NOT
 *   LIMITED TO,  PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES;  LOSS OF USE,
 *   DATA, OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED  AND ON  ANY
 *   THEORY  OF  LIABILITY,  WHETHER IN  CONTRACT,  STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY  WAY  OUT OF  THE USE
 *   OF THIS SOFTWARE, EVEN  IF  ADVISED OF  THE POSSIBILITY  OF SUCH DAMAGE.
 */

#ifndef xeumeuleu_caller0_hpp
#define xeumeuleu_caller0_hpp

namespace xml
{
    class xistream;

// =============================================================================
/** @class  caller0
    @brief  Method call functor
*/
// Created: MAT 2006-01-06
// =============================================================================
template< typename T >
class caller0
{
private:
    //! @name Types
    //@{
    typedef void (T::*M)( xistream& );
    //@}

public:
    //! @name Constructors/Destructor
    //@{
    template< typename I >
    caller0( I& instance, M method )
        : method_  ( method )
        , instance_( instance )
    {}
    //@}

    //! @name Operations
    //@{
    void operator()( xistream& xis ) const
    {
        (instance_.*method_)( xis );
    }
    //@}

private:
    //! @name Constructors/Destructor
    //@{
    caller0& operator=( const caller0& ); //!< Assignment operator
    //@}

private:
    //! @name Member data
    //@{
    M method_;
    T& instance_;
    //@}
};

// =============================================================================
/** @class  const_caller0
    @brief  Const method call functor
*/
// Created: MAT 2006-01-06
// =============================================================================
template< typename T >
class const_caller0
{
private:
    //! @name Types
    //@{
    typedef void (T::*M)( xistream& ) const;
    //@}

public:
    //! @name Constructors/Destructor
    //@{
    template< typename I >
    const_caller0( const I& instance, M method )
        : method_  ( method )
        , instance_( instance )
    {}
    //@}

    //! @name Operations
    //@{
    void operator()( xistream& xis ) const
    {
        (instance_.*method_)( xis );
    }
    //@}

private:
    //! @name Constructors/Destructor
    //@{
    const_caller0& operator=( const const_caller0& ); //!< Assignment operator
    //@}

private:
    //! @name Member data
    //@{
    M method_;
    const T& instance_;
    //@}
};

}

#endif // xeumeuleu_caller0_hpp
