#include "matplotlibcpp.h"
#include <vector>

namespace plt = matplotlibcpp;

int main()
{
    std::vector<double> x = {1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> y1 = {2.0, 4.5, 6, 4.5, 2.0};
    std::vector<double> y2 = {1.0, 3.0, 5.0, 7.0, 9.0};

    plt::subplot(2, 1, 1);
    plt::plot(x, y1);
    plt::xlabel("X");
    plt::ylabel("Y1");

    plt::subplot(2, 1, 2);
    plt::plot(x, y2);
    plt::xlabel("X");
    plt::ylabel("Y2");

    plt::suptitle("Subplots");

    plt::save("subplot.png");  // Save the plot to a file

    plt::show();

    return 0;
}