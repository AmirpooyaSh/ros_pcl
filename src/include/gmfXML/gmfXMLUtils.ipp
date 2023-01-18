/////////////////////////////////////////////////
/////////////////////////////////////////////////
//
// The gmfXML library (including all source
// and header files, examples, and
// documentation) is covered under
// copyright (c) 2008 by Matthew Peavy
// of Give Me Fish, LLC and is
// licensed under the LGPL 3.0 licence
// from June 29, 2007.
//
// Please see license documents within
// the License directory or go to 
// http://www.fsf.org/licensing to obtain
// a copy of the LGPL license.
//
/////////////////////////////////////////////////


void GiveMeFish::XMLlib::loadFromStream(std::string& fileText, std::istream& resultsFile)
{
   fileText.erase();
   if(resultsFile.bad()) 
      return;
   fileText.reserve(resultsFile.rdbuf()->in_avail());
   char c;
   while(resultsFile.get(c))
   {
      if(fileText.capacity() == fileText.size())
         fileText.reserve(fileText.capacity() * 2);
      fileText.append(1, c);
   }
}

void GiveMeFish::XMLlib::stringReplaceInstances(std::string& source, const std::string& from, const std::string& to)
{
  std::string newString;
  newString.reserve(source.length());  //Avoids extra memory allocations

  std::string::size_type lastPos = 0;
  std::string::size_type findPos;

  while(std::string::npos != (findPos = source.find(from, lastPos))) {
      newString.append(source, lastPos, findPos - lastPos);
      newString += to;
      lastPos = findPos + from.length();
  }

  //Do the rest after the last occurrence
  newString += source.substr(lastPos);
  source.swap(newString);
}
