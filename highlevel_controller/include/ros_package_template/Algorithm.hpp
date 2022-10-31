#pragma once

namespace ros_package_template {

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
