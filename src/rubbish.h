#pragma once


#if false
template <class _Corpus>
class Mat {
public:
	using area = std::vector<std::vector<_Corpus>>;
	area m;

	Mat(std::size_t m, std::size_t n) : m(area(std::vector<_Corpus>(_Corpus(0), n), m)) {
	}
};

template<class _Corpus>
Mat<_Corpus> enhanced_solve_linear_system(Mat<_Corpus> A, Mat<_Corpus> b, Mat<bool> target) {
	/*
	Ax = b
	A: m x n
	b: n x 1
	x: m x 1
	ready: m x 1
	*/

	// check:
	A.area.size() != 0;
	A.area[0].size() == b.area.size();
	ready.area.size() == A.area.size();

	Mat<bool> ready = target;

}

#endif

