// src: http://stackoverflow.com/questions/23400933/most-efficient-way-of-creating-a-progress-bar-while-reading-input-from-a-file

# include <cmath>
# include <string>
# include <fstream>
# include <iomanip>
# include <iostream>

class ProgressBarBase
{
  public:

    ProgressBarBase(int max) : max_( max )
    { print(0); }

  protected:

    // Prints out the progress bar for a progress value between 0 and 1.
    inline void print(float prog)
    {
      assert( 0 <= prog and prog <= 1 );

      int cur( std::ceil(prog * max_) );

      std::cout << std::fixed << std::setprecision(2)
        << "\r   [" << std::string(cur, '#')
        << std::string(max_ + 1 - cur, ' ') << "] " << 100 * prog << "%";

      if (prog == 1)
        std::cout << std::endl;
      else
        std::cout.flush();
    }

  private:

    // maximum number of markers to print.
    int max_;
};

class ProgressBar : public ProgressBarBase
{
  public:

    ProgressBar(int max) : ProgressBarBase( max ) {}

    inline void drawbar(float prog)
    { print( prog ); }

};

class pretty_ifstream : public std::ifstream, public ProgressBarBase
{
  public:

    template <class... Args>
    inline pretty_ifstream(int max, Args&&... args) :
    std::ifstream(args...), ProgressBarBase( max )
    {
      if (std::ifstream::is_open()) _measure();
    }

    // Opens the file and measures its length
    template <class... Args>
    inline auto open(Args&&... args)
    -> decltype(std::ifstream::open(args...))
    {
      auto rvalue(std::ifstream::open(args...));
      if (std::ifstream::is_open()) _measure();
        return rvalue;
    }

    // Displays the progress bar (pos == -1 -> end of file)
    inline void drawbar(void)
    {
      int pos(std::ifstream::tellg());

      // percentage of infile already read
      float prog(pos / float(_length));

      // this gets called at the end, because when we have read
      // the whole file, tellg returns -1
      if (pos == -1)
      {
        print( 1 );
        return;
      }

      // Number of #'s as function of current progress "prog"
      print( prog );
    }

  private:

    std::string _inpath;
    int _length;

    // Measures the length of the input file
    inline void _measure(void)
    {
        std::ifstream::seekg(0, end);
        _length = std::ifstream::tellg();
        std::ifstream::seekg(0, beg);
    }
};
