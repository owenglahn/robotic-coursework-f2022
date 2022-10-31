#pragma once

namespace highlevel_controller {

/*!
 * Class containing the algorithmic part of the package.
 */
class Algorithm {

	public:

	/*!
	* Constructor.
	*/
	Algorithm();

	/*!
	* Destructor.
	*/
	virtual ~Algorithm();

	/*!
	* a function that runs some algorithms
	*/
	void update ();


	private:


};

} /* namespace */
