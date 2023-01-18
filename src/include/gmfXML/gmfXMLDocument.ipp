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

GiveMeFish::XMLlib::XMLDocument::XMLDocument()
{
}

GiveMeFish::XMLlib::XMLDocument::XMLDocument(std::istream& in)
{
  //Don't skip white-spaces
  in >> std::noskipws;

  //Load the input stream into a string
  std::string text;
  loadFromStream(text, in);

  //Now initialize from that string
  init(text.begin(), text.end());
}

GiveMeFish::XMLlib::XMLDocument::XMLDocument(std::string::const_iterator start, std::string::const_iterator end)
{
  init(start, end);
}

GiveMeFish::XMLlib::XMLDocument::XMLDocument(const std::vector<std::string>& processingInstructions, const XMLElement& root)
	: m_processingInstructions(processingInstructions), m_root(root)
{
}

void GiveMeFish::XMLlib::XMLDocument::init(std::string::const_iterator start, const std::string::const_iterator& end)
{
   //Parse recursively until all sub-nodes have been read.

   //First set up the "document-level" regular expression
   boost::regex regDoc(XML_SPE); 
  
   //This holds the regex match
   boost::match_results<std::string::const_iterator> results1;
   boost::match_flag_type flags = boost::match_default;

   //Loop so long as there are matches. There may be more than 1 processing instruction.
   while(regex_search(start, end, results1, regDoc, flags)) 
   { 
      std::string token(results1[0].str());
      
	  boost::match_results<std::string::const_iterator> results2;
	  boost::regex regMarkup(MarkupSPE);
      if(boost::regex_match(token, results2, regMarkup)) {

         //There should be processing instructions or an element.
         //If not, this is an error.
         if(handleIfElement(token, start, end)) {
            //We're done if we handled this element successfully
            return;
         }
         
         //Note, the start iterator will be incremented.
         if(!handleIfProcessingInstruction(token, start)) {
            //This is an error.
			 std::stringstream ss;
            ss << "***ERROR - Invalid token:  " << token << std::endl;
            throw(gmfXMLException(ss.str()));
         }
      }
      else {
         //Else we are just skipping garbage. 
         //Only thing to do is advance the iterator
         start = results1[0].second; 
      }
   } 
}

void GiveMeFish::XMLlib::XMLDocument::clear()
{
  m_processingInstructions.clear();
  m_root.clear();
}

////////// PRIVATE //////////////

void GiveMeFish::XMLlib::XMLDocument::print(std::ostream& out, unsigned numTabs, const std::string& tab, const std::string& newLine) const
{
   //Print processing instructions first (if any)
   for(std::vector<std::string>::const_iterator vi = m_processingInstructions.begin();
       vi != m_processingInstructions.end(); ++vi)
   {
       out << *vi << std::endl;
   }      

   //Now print the root element
   out << m_root;
}
    
bool GiveMeFish::XMLlib::XMLDocument::handleIfElement(const std::string& token, std::string::const_iterator start, const std::string::const_iterator& end)
{
   boost::match_results<std::string::const_iterator> resultsElem; 
   boost::regex regElement(ElemTag_SPE);
   if(boost::regex_match(token, resultsElem, regElement)) {

      //We found an element match. Initialize the root element.
      m_root.init(start, end);

      //Return true since we handled an Document
      return true;
   }
   else
      //Return false since we didn't handle a Document
      return false;
}

bool GiveMeFish::XMLlib::XMLDocument::handleIfProcessingInstruction(const std::string& token, std::string::const_iterator& start)
{
   //Check if there is a processing instsruction.  If so, add it to the vector.
	boost::match_results<std::string::const_iterator> results;
	boost::regex regPI(PI_SPE);
   if(regex_match(token, results, regPI)) {
      m_processingInstructions.push_back(results.str());

      //Also, must increment the start iterator.
      start += results.length();

      //Return true since we handled a processing instruction.
      return true;
   }
   else
      return false;
}


//////////// Free Functions /////////

std::ostream& operator<<(std::ostream& out, const GiveMeFish::XMLlib::XMLDocument& doc)
{
  doc.print(out, 0, "\t", "\n");
  return out;
}


