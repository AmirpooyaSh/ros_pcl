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


namespace GiveMeFish {
	namespace XMLlib {

		//Only need to construct these once.
		//Implicit perl regex type:   regex::perl
		const static boost::regex regXML_SPE(GiveMeFish::XMLlib::XML_SPE);
		const static boost::regex regElemTag_SPE(GiveMeFish::XMLlib::ElemTag_SPE);
		const static boost::regex regComment_SPE(GiveMeFish::XMLlib::Comment_SPE);
	}
}

bool GiveMeFish::XMLlib::skipWSs(std::string::const_iterator& start, std::string::const_iterator end)
{
	//Skip white spaces
	while ((*start == ' ') || (*start == '\t') ||
		   (*start == '\n') || (*start == '\r'))
	{
		++start;
		if (start == end)
			return false;
	}
	return true;
}

bool GiveMeFish::XMLlib::containsOnlyWSs(const std::string& token)
{
	//Returns true if the string only contains white spaces
	std::string::const_iterator start = token.begin();
	skipWSs(start, token.end());
	return(start == token.end());
}

void GiveMeFish::XMLlib::printTabs(std::ostream& out, unsigned numTabs, const std::string& tab)
{
	//Prints tabs to the stream passed in
	for (unsigned i = 0; i < numTabs; ++i)
		out << tab;
}

void GiveMeFish::XMLlib::replaceAmpChars(std::string& tempVal)
{
	stringReplaceInstances(tempVal, "&", "&amp;");
	stringReplaceInstances(tempVal, "<", "&lt;");
	stringReplaceInstances(tempVal, ">", "&gt;");
}

void GiveMeFish::XMLlib::reinsertAmpChars(std::string& tempVal)
{
	stringReplaceInstances(tempVal, "&gt;", ">");
	stringReplaceInstances(tempVal, "&lt;", "<");
	stringReplaceInstances(tempVal, "&amp;", "&");
}

///////////////

GiveMeFish::XMLlib::XMLElement::XMLElement()
: m_isValidXML(false), m_isComment(false)
{
}

GiveMeFish::XMLlib::XMLElement::XMLElement(const std::string& tag)
: m_isValidXML(true), m_isComment(false), m_tag(tag)
{
}

GiveMeFish::XMLlib::XMLElement::XMLElement(const std::string& tag, const std::string& value)
: m_isValidXML(true), m_isComment(false), m_tag(tag), m_value(value)
{
	//Be sure to replace any "&" symbols back to their original from
	reinsertAmpChars(m_value);
}

GiveMeFish::XMLlib::XMLElement::XMLElement(std::string::const_iterator& start, const std::string::const_iterator& end)
: m_isValidXML(true), m_isComment(false)
{
  init(start, end);
}

GiveMeFish::XMLlib::XMLElement::XMLElement(std::istream& in)
: m_isValidXML(true), m_isComment(false)
{
  //Don't skip white-spaces
  in >> std::noskipws;

  //Load the input stream into a std::string
  std::string text;
  loadFromStream(text, in);

  //Now initialize from that std::string
  std::string::const_iterator start = text.begin();
  init(start, text.end());
}

void GiveMeFish::XMLlib::XMLElement::init(std::string::const_iterator& start, const std::string::const_iterator& end)
{
   //We want to get out the tag name and then parse for a value or (possibly multiple) sub-elements.
   //First match on the element
   boost::match_results<std::string::const_iterator> resultsElem;         //aka: smatch
   boost::regex_search(start, end, resultsElem, regElemTag_SPE);
     
   //Parse the opening tag.  This may include attributes.
   std::string openTag = resultsElem.str();
   std::string::const_iterator openTagBegin = openTag.begin();
   std::string::const_iterator openTagEnd = openTag.end();
   parseOpenTag(openTagBegin, openTagEnd);

   //Increment the start iterator to just past the element
   start += resultsElem.length();

   //Determine if this element has sub-elements or contains a value.
   //First pull out the next XML piece. 
   //We're looking for either a full sub-element open tag or the value for this element.
   //Note - the XML_SPE regex will match on pure whitespace preceeding an open tag.
   //We can't just skip white-spaces, however, because if we're parsing a value and 
   // it contains white-spaces as the first character(s), we will skip them, causing
   // the value to not be parsed faithfully.
   //So, the methodology is as follows:
   // *) Search using XML_SPE.  
   // *) If the resulting markup contains all whitespaces, 
   //      Attempt a match on SPE or comment.
   //      If it is one of these, use it (the new markup and start position).
   // *) Else, use whatever was originally matched.
   boost::match_results<std::string::const_iterator> resultsSPE;
   boost::regex_search(start, end, resultsSPE, regXML_SPE);
   std::string markup(resultsSPE.str());
   if(containsOnlyWSs(markup)) {
      std::string::const_iterator tempStart(start);
      tempStart += resultsSPE.length();
      regex_search(tempStart, end, resultsSPE, regXML_SPE);
      std::string tempMarkup(resultsSPE.str());
      if(regex_match(tempMarkup, resultsSPE, regElemTag_SPE) || 
         regex_match(tempMarkup, resultsSPE, regComment_SPE))  {
         //This is an element or comment preceeded by spaces. Use it
         // by updating the markup and start position.
         markup = tempMarkup;
         start = tempStart;
      }
      //else, use the existing markup and start position.
   }

   //Now try to match the mark-up to an element or a comment. If it matches, then we have a sub-element or a comment.
   //Both elements and comments are parsed in parseSubElements function. Otherwise parse out the value.
   // $FUTURE UPDATE - combine the element and comment regex into one expression
   boost::regex regEndTag(EndTag_SPE);
   boost::match_results<std::string::const_iterator> resultsSubElement;
   if(boost::regex_match(markup, resultsSubElement, regElemTag_SPE)) {
      //Since we matched on an element, parse out the sub-elements.
      parseSubElements(start, end);
   } 
   else if(boost::regex_match(markup, resultsSubElement, regComment_SPE)) {
      //Since we matched on a comment, parse out the sub-elements.
      parseSubElements(start, end);
   }
   else if(boost::regex_match(markup, resultsSubElement, regEndTag)) {
      //Since we matched on an end tag, there was a blank value. This
      // is acceptable.  m_value defaults to "".
      // Only thing to do is increment start iterator to past the end tag
      start += resultsSubElement.length();
   }
   else {
      //Since we didn't match, parse out the value.
      parseValue(start, end, markup);
   }

   //If we have successfully init'd, then set this to valid XML
   m_isValidXML = true;
}

const GiveMeFish::XMLlib::XMLElement& GiveMeFish::XMLlib::XMLElement::getNode(std::vector<XMLElement>::size_type index) const
{
   if(index >= m_subNodes.size())
      throw(gmfXMLException("***Error - getNode indexing error: out of bounds"));

	return m_subNodes[index];
}

void GiveMeFish::XMLlib::XMLElement::clear()
{
  m_isValidXML =false;
  m_isComment =false;
  m_tag ="";
  m_value ="";
  m_attributes.clear();
  m_subNodes.clear();
}

////////// PRIVATE //////////////

void GiveMeFish::XMLlib::XMLElement::print(std::ostream& out, unsigned numTabs, const std::string& tab, const std::string& newLine) const
{
  printTabs(out, numTabs, tab);

  if(m_isComment)
     out << "<!--";
  else
     out << "<" << m_tag;

   //Print attributes (if necessary)
   for(Attributes::const_iterator ai = m_attributes.begin(); ai !=m_attributes.end(); ++ai) {
      out << " " << ai->first << "=" << '"' << ai->second << '"';
   }   

   //Close the tag
   if(!m_isComment)
	 out << ">";

  //If there are no childern, print the value (even if it's empty) and then close the element
  if(m_subNodes.size() == 0) {

	//We must replace all &, <, and > characters when printing.
	std::string tempValue(m_value);
	replaceAmpChars(tempValue);
    out << tempValue;
    if(m_isComment)
       out << "-->";
    else
       out << "</" << m_tag << ">";
  }
  else {
    //Otherwise iterate through all the childern and print them.
    for(std::vector<XMLElement>::const_iterator ni = m_subNodes.begin(); ni != m_subNodes.end(); ++ni) {
      out << newLine;
      ni->print(out, numTabs+1, tab);
    }
    out << newLine;
    printTabs(out, numTabs, tab);
    if(m_isComment)
       out << "-->";
    else
       out << "</" << m_tag << ">";
  }
}
  
bool GiveMeFish::XMLlib::XMLElement::handleIfComment(const std::string& token, std::string::const_iterator& start)
{
   boost::match_results<std::string::const_iterator> results; 
   if(boost::regex_match(token, results, regComment_SPE)) {
      std::string commentValue = results.str().substr(4, results.length()-7);
      XMLElement commentElement("", commentValue); 
      commentElement.m_isComment = true;
      m_subNodes.push_back(commentElement);

      //Also, must increment the start iterator, and then return true.
      start += results.length();
      return true;
   }
   else
      return false;
}
    
bool GiveMeFish::XMLlib::XMLElement::handleIfSubElement(const std::string& token, std::string::const_iterator& start, const std::string::const_iterator& end)
{
	boost::match_results<std::string::const_iterator> resultsElem;
   if(boost::regex_match(token, resultsElem, regElemTag_SPE)) {
      XMLElement subElement(start, end);
      m_subNodes.push_back(subElement);

      //Return true since we handled a sub-element
      return true;
   }
   else
      //Return false since we didn't handle an element
      return false;
}
    
bool GiveMeFish::XMLlib::XMLElement::handleIfCloseTag(const std::string& token, std::string::const_iterator& start)
{
	boost::match_results<std::string::const_iterator> resultsEndTag;
	boost::regex regEndTag(EndTag_SPE);
   if(regex_match(token, resultsEndTag, regEndTag)) {

      std::string endTag(resultsEndTag.str());

      //Copy out the tag name and store it.
	  boost::match_results<std::string::const_iterator> resultsName;
	  boost::regex regName(Name);
      assert(regex_search(token, resultsName, regName)); //This should be guaranteed
      regex_search(token, resultsName, regName);

      //Verify that the end tag matches this tag
      if(m_tag != resultsName.str()) {
         //This is an error.
         std::stringstream ss;
         ss << "***ERROR - Closing name error: " << m_tag << " != " << resultsName.str();
         throw(gmfXMLException(ss.str()));
      }

      //Increment the start iterator to just past the token we parsed out, and return true.
      start += token.size();
      return true;
   }
   else
      return false;
}

void GiveMeFish::XMLlib::XMLElement::parseOpenTag(std::string::const_iterator& start, const std::string::const_iterator& end)
{
  //This function will copy out the tag name and any attributes (possibly 0, 1, or more than 1)
  // and store them in m_attributes.

  //First copy out the tag name and store it.
	boost::match_results<std::string::const_iterator> resultsName;
	boost::regex regName(Name);
  assert(regex_search(start, end, resultsName, regName)); //This should be guaranteed
  boost::regex_search(start, end, resultsName, regName);
  m_tag = resultsName.str();

  //Handle potential attributes here. They are of the form: attName = "attValue", which 
  // correspond to pairs of REX regex values:  Name / AttValSE
  //There may be 0, 1, or many such pairs.
  //Note AttValSE will return the value as a quoted string, eg "attValue" though we
  // will want to store it as an unquoted string.
  
  start += (resultsName.position() + resultsName.length());   //Advance past the tag
  boost::match_results<std::string::const_iterator> resultsAttributeName;
  boost::regex regAttValSE(AttValSE);
  while(boost::regex_search(start, end, resultsAttributeName, regName)) {

    start += (resultsAttributeName.position() + resultsAttributeName.length());  //Advance past the attribute name
	boost::match_results<std::string::const_iterator> resultsAttributeValue;
    if(!boost::regex_search(start, end, resultsAttributeValue, regAttValSE)) {
	  //Should have an attribute value if there is an attribute name. This is an error.
	  std::stringstream ss;
	  ss << "***ERROR - attribute name missing value, for element: " << m_tag << ", attribute " << resultsAttributeName.str();
	  throw(gmfXMLException(ss.str()));
    }

    //Strip the quotes off the attribute value
    std::string unquotedAttribute(resultsAttributeValue.str());
    boost::replace_all(unquotedAttribute, singleQuote, "");

    //Insert into the attributes vector
    m_attributes.push_back(Attribute(resultsAttributeName.str(), unquotedAttribute));

    //Update the start position to get past this set of attributes
    start += (resultsAttributeValue.position() + resultsAttributeValue.length());
  }
}
      
void GiveMeFish::XMLlib::XMLElement::parseSubElements(std::string::const_iterator& start, const std::string::const_iterator& end)
{
  //This element contains one or more sub-elements.  Loop over them.
	boost::match_results<std::string::const_iterator> resultsSubElems;
	boost::match_flag_type flags = boost::match_default;

  //Loop so long as there are matches
  while(boost::regex_search(start, end, resultsSubElems, regXML_SPE, flags)) {
	std::string token(resultsSubElems.str());
   
	if(handleIfComment(token, start)) {
	}
	else if(handleIfSubElement(token, start, end)) {
	}
	else if(handleIfCloseTag(token, start)) {
	  // Close tag.  If we successfully handled the end tag, then we're done with this element
	  return;
	}
	else {
	  //This is an error.
	  std::stringstream ss;
	  ss << "***ERROR - Invalid token:  " << token << std::endl;
	  throw(gmfXMLException(ss.str()));
	}
	  
	//Skip any leading white spaces
	skipWSs(start, end);
  }
}

void GiveMeFish::XMLlib::XMLElement::parseValue(std::string::const_iterator& start, const std::string::const_iterator& end, const std::string& markup)
{
   //Else we have a value for this element (and not sub-elements).
   m_value = markup;

   //Be sure to replace any "&" symbols back to their original from
   reinsertAmpChars(m_value);
      
   //Increment past the value
   start += markup.size();

   //And now we should read out the close tag
   boost::match_results<std::string::const_iterator> resultsElem;
   boost::regex_search(start, end, resultsElem, regXML_SPE);
   if(!handleIfCloseTag(resultsElem.str(), start)) {
      //This is an error.
      throw(gmfXMLException("***ERROR - Close tag not found while parsing value."));
   }
}

//////////// Free Functions /////////

std::ostream& operator<<(std::ostream& out, const GiveMeFish::XMLlib::XMLElement& node)
{
  node.print(out, 0, "\t", "\n");
  return out;
}

