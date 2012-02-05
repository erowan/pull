
import lxml.html
import urlparse
import urllib
 
class BBCNews(object):
    
    url = 'http://www.bbc.com/news/'
    
    def __init__(self):
        html = urllib.urlopen(self.url).read() 
        self.page = lxml.html.fromstring(html)
                 
    def parse_headline(self):        
        div = self.page.cssselect('div#top-story')[0]
        a = div.cssselect('h2.top-story-header a')[0]
        href = urlparse.urljoin(self.url, a.attrib.get('href'))
        headline = str(a.text_content())                  
        summary = div.cssselect('p')[0].text_content()
        return href, headline, summary            
    
    def find_paragraph(self, word):
        paragraphs = [x.text_content() for x in self.page.cssselect('p')]  
        result = []
        if isinstance(word, basestring):
            result = [x for x in paragraphs if word in x]
        else:
            result = [x for x in paragraphs if set(x.split()).issuperset(set(word))]
        return result       

def main(): 
    bbc = BBCNews()
    print bbc.parse_headline()  
    print bbc.find_paragraph('always')
    print bbc.find_paragraph(('Cuts', 'defence'))  
   
if __name__ == "__main__":
    main()



      
