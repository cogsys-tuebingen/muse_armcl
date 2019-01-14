#ifndef CONFUSION_MATRIX_HPP
#define CONFUSION_MATRIX_HPP
#include <vector>
#include <map>
#include <set>
#include <fstream>

class ConfusionMatrix
{
public:
    ConfusionMatrix() {}

    void reportClassification(int actual, int prediction)
    {
        if (classes_set.count(actual) == 0) {
            initializeClass(actual);
        }
        if (classes_set.count(prediction) == 0) {
            initializeClass(prediction);
        }

        ++histogram[std::make_pair(actual, prediction)];
    }


    void initializeClass(int _class)
    {
        classes.push_back(_class);
        classes_set.insert(_class);

        resetClass(_class);
    }

    void reset()
    {
        for (auto c : classes) {
            resetClass(c);
        }
    }

    void exportCsv(const std::string& file)
    {
        if(classes.empty()){
            return;
        }
        std::ofstream of(file);

//        std::cout << file << std::endl;

        // row = prediction, col = actual
        std::sort(classes.begin(), classes.end());

        of << "classes";
        for (int cl : classes) {
            of << ',' << cl;
        }
        of << '\n';

        for (int prediction : classes) {
            of << prediction;
            for (int actual : classes) {
                of << ',' << histogram.at(std::make_pair(actual, prediction));
            }
            of << '\n';
        }

        of.close();
    }

public:
    std::vector<int> classes;
    std::map<int, std::string> class_names;
    std::map<std::pair<int, int>, int> histogram;

private:
    void resetClass(int _class)
    {
        for (std::vector<int>::const_iterator it = classes.begin(); it != classes.end(); ++it) {
            histogram[std::make_pair(*it, _class)] = 0;
            histogram[std::make_pair(_class, *it)] = 0;
        }
    }

private:
    std::set<int> classes_set;
};
#endif // CONFUSION_MATRIX_HPP
