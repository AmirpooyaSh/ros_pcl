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

#ifndef GIVEMEFISH_XMLLIB_GMFXMLDOCUMENT_H
#define GIVEMEFISH_XMLLIB_GMFXMLDOCUMENT_H

//boost
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>   //For the replace_all call
//std
#include <sstream>
#include <cassert>
#include <iostream>
#include <vector>
#include <string>
#include <iterator>

// Recommend using a namespace alias in client code. For example:
//
//  namespace gmfXML = GiveMeFish::XMLlib;
//

namespace GiveMeFish {

   namespace XMLlib {


	  // NOTE: This file contains code obtained from
	  // the REX library. See copyright below.

	  // REX
	  // Robert D. Cameron "REX: XML Shallow Parsing with Regular Expressions",
	  // Technical Report TR 1998-17, School of Computing Science, Simon Fraser 
	  // University, November, 1998.
	  // Copyright (c) 1998, Robert D. Cameron. 
	  // The following code may be freely used and distributed provided that
	  // this copyright and citation notice remains intact and that modifications
	  // or additions are clearly identified.


	  /////////////////////////////////////////////////
	  //
	  // REX was modified and added to by Matthew D. Peavy
	  // of Give Me Fish, LLC (hereafter: MDP) in the 
	  // following manner:
	  //
	  // *) Adapted REX to work in C++
	  // *) Placed within the GiveMeFish::XMLlib namespace
	  // *) Added the const singleQuote where appropriate
	  //
	  // Originally found here: http://www.cs.sfu.ca/~cameron/REX.html
	  // 

	  const std::string singleQuote("\""); //$ADDED MDP
	  const std::string TextSE("[^<]+");
	  const std::string UntilHyphen("[^-]*-");
	  const std::string Until2Hyphens(UntilHyphen + "([^-]" + UntilHyphen + ")*-");
	  const std::string CommentCE(Until2Hyphens + ">?");
	  const std::string UntilRSBs("[^]]*]([^]]+])*]+");
	  const std::string CDATA_CE(UntilRSBs + "([^]>]" + UntilRSBs + ")*>");
	  const std::string S("[ \\n\\t\\r]+");
	  const std::string NameStrt("[A-Za-z_:]|[^\\x00-\\x7F]");
	  const std::string NameChar("[A-Za-z0-9_:.-]|[^\\x00-\\x7F]");
	  const std::string Name("(" + NameStrt + ")(" + NameChar + ")*");
	  const std::string QuoteSE(singleQuote + "[^" + singleQuote + "]" + "*" + singleQuote + "|'[^']*'");
	  const std::string DT_IdentSE(S + Name + "(" + S + "(" + Name + "|" + QuoteSE + "))*");
	  const std::string MarkupDeclCE("([^]\"'><]+|" + QuoteSE + ")*>");
	  const std::string S1("[\\n\\r\\t ]");
	  const std::string UntilQMs("[^?]*\\?+");
	  const std::string PI_Tail("\\?>|" + S1 + UntilQMs + "([^>?]" + UntilQMs + ")*>");
	  const std::string DT_ItemSE("<(!(--" + Until2Hyphens + ">|[^-]" + MarkupDeclCE + ")|\\?" + Name + "(" + PI_Tail + "))|%" + Name + ";|" + S);
	  const std::string DocTypeCE(DT_IdentSE + "(" + S + ")?(\\[(" + DT_ItemSE + ")*](" + S + ")?)?>?");
	  const std::string DeclCE("--(" + CommentCE + ")?|\\[CDATA\\[(" + CDATA_CE + ")?|DOCTYPE(" + DocTypeCE + ")?");
	  const std::string PI_CE(Name + "(" + PI_Tail + ")?");
	  const std::string EndTagCE(Name + "(" + S + ")?>?");
	  const std::string AttValSE(singleQuote + "[^<" + singleQuote + "]" + "*" + singleQuote + "|'[^<']*'");
	  const std::string ElemTagCE(Name + "(" + S + Name + "(" + S + ")?=(" + S + ")?(" + AttValSE + "))*(" + S + ")?/?>?");
	  const std::string MarkupSPE("<(!(" + DeclCE + ")?|\\?(" + PI_CE + ")?|/(" + EndTagCE + ")?|(" + ElemTagCE + ")?)");
	  const std::string XML_SPE(TextSE + "|" + MarkupSPE);
	  const std::string PI_SPE("<\\?(" + PI_CE + ")?");  //Processing Instruction Shallow Parse Document
	  const std::string Comment_SPE("<!--(" + CommentCE + ")?");  //Comment Shallow Parse Document
	  const std::string ElemTag_SPE("<" + ElemTagCE);
	  const std::string EndTag_SPE("</(" + EndTagCE + ")?");
	  const std::string Comment_RE("<!--" + Until2Hyphens + ">");

	  //////////////////////////

	  class gmfXMLException : public std::exception {
	  public:
		  gmfXMLException() {};
		  explicit gmfXMLException(const std::string& what) : m_what(what) {};
		  ~gmfXMLException() throw() {};

		  const std::string& getWhat() const { return m_what; }
		  void print(std::ostream& out) const { out << m_what; }

	  private:
		  std::string m_what;
	  };

	  ////////////////////

	  inline bool skipWSs(std::string::const_iterator& start, std::string::const_iterator end);
	  inline bool containsOnlyWSs(const std::string& token);
	  inline void printTabs(std::ostream& out, unsigned numTabs, const std::string& tab);
	  inline void replaceAmpChars(std::string& tempVal);
	  inline void reinsertAmpChars(std::string& tempVal);

	  class XMLElement {
	  public:
		  //typedefs for XML elements
		  typedef std::vector<XMLElement>::iterator iterator;
		  typedef std::vector<XMLElement>::const_iterator const_iterator;
		  typedef std::pair<std::string, std::string> Attribute;
		  typedef std::vector<Attribute> Attributes;

		  inline XMLElement();
		  inline explicit XMLElement(const std::string& tag);
		  inline XMLElement(const std::string& tag, const std::string& value);
		  inline XMLElement(std::string::const_iterator& start, const std::string::const_iterator& end); //Start iterator will be incremented after construction
		  inline explicit XMLElement(std::istream&);

		  inline void init(std::string::const_iterator& start, const std::string::const_iterator& end); //Start iterator will be incremented after init
		  inline void clear();
		  inline bool empty() const { return (m_subNodes.size() == 0); }
		  inline bool isValidXML() const { return m_isValidXML; }
		  inline bool isComment() const { return m_isComment; }
		  inline std::vector<XMLElement>::size_type size() const { return m_subNodes.size(); }

		  inline iterator begin() { return m_subNodes.begin(); }
		  inline const_iterator begin() const { return m_subNodes.begin(); }
		  inline iterator end() { return m_subNodes.end(); }
		  inline const_iterator end() const { return m_subNodes.end(); }

		  inline const XMLElement& getNode(std::vector<XMLElement>::size_type index) const;

		  inline const std::string& getValue() const { return m_value; }
		  inline const std::string& getTag() const { return m_tag; }
		  inline const Attributes& getAttributes() const { return m_attributes; }
		  inline void  setAttribute(const Attribute& attribute) { m_attributes.clear(); m_attributes.push_back(attribute); }
		  inline void  setAttributes(const Attributes& attributes) { m_attributes = attributes; }

		  inline void appendSubNode(const XMLElement& elem) { m_subNodes.push_back(elem); }

		  inline void print(std::ostream&, unsigned numTabs = 0,
                        const std::string & tab = "\t", const std::string & newLine = "\n") const;

	  private:
		  bool                    m_isValidXML;
		  bool                    m_isComment;    //Denotes that this Element is a special comment element.
		  std::string             m_tag;
		  std::string             m_value;
		  Attributes              m_attributes;
		  std::vector<XMLElement> m_subNodes;

		  inline bool handleIfComment(const std::string& token, std::string::const_iterator& start);  //start will be incremented
		  inline bool handleIfSubElement(const std::string&, std::string::const_iterator& start, const std::string::const_iterator& end);
		  inline bool handleIfCloseTag(const std::string&, std::string::const_iterator& start);
		  inline void parseOpenTag(std::string::const_iterator& start, const std::string::const_iterator& end);
		  inline void parseSubElements(std::string::const_iterator& start, const std::string::const_iterator& end);
		  inline void parseValue(std::string::const_iterator& start, const std::string::const_iterator& end, const std::string& markup);
	  };

	  typedef XMLElement XMLRootElement;

	  ////////////////////////////

	  class XMLDocument {
	  public:
		  inline XMLDocument();
		  inline XMLDocument(std::istream&);
		  inline XMLDocument(std::string::const_iterator start, std::string::const_iterator end);
		  inline XMLDocument(const std::vector<std::string>& processingInstructions, const XMLElement& root);
		  
		  inline void init(std::string::const_iterator start, const std::string::const_iterator& end);
		  inline void clear();
		  
		  inline bool isValidXML() const { return m_root.isValidXML(); }
		  inline const std::vector<std::string>& getProcessingInsturctions() const { return m_processingInstructions; }
		  inline const XMLElement& getRootElement() const { return m_root; }
		  inline std::string getRootTag() const { return m_root.getTag(); }
		  
		  inline void print(std::ostream&, unsigned numTabs = 0,
			  const std::string & tab = "\t", const std::string & newLine = "\n") const;
		  
	  private:
		  std::vector<std::string> m_processingInstructions;
		  XMLElement m_root;
		  
		  inline bool handleIfProcessingInstruction(const std::string& token, std::string::const_iterator& start);  //start will be incremented
		  inline bool handleIfElement(const std::string& token, std::string::const_iterator start, const std::string::const_iterator& end);
	  };
	  
	  //////////////////
	  
      //Load a string based on a stream
	  inline void loadFromStream(std::string& fileText, std::istream& resultsFile);

	  //Replace any number of instances in-place in the source string
      // of the 'from' string into the 'to' string
	  inline void stringReplaceInstances(std::string& source, const std::string& from, const std::string& to);

   }
}

//Implemented in terms of print()
inline std::ostream& operator<<(std::ostream&, const GiveMeFish::XMLlib::gmfXMLException&);
inline std::ostream& operator<<(std::ostream&, const GiveMeFish::XMLlib::XMLElement&);
inline std::ostream& operator<<(std::ostream&, const GiveMeFish::XMLlib::XMLDocument&);

//Include the implementation files
#include "gmfXMLUtils.ipp"
#include "gmfXMLElement.ipp"
#include "gmfXMLDocument.ipp"

#endif

