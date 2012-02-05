.. pull documentation master file, created by
   sphinx-quickstart on Sat Feb  4 16:25:54 2012.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

pull - A web scraper library
================================

A scrape or feed can be broken down into 3 steps: Fetch->Parse->Store. The 
library defines a scaffolding that will run these steps for a feed or 
collection of feeds. A user needs to define the steps with the classes 
provided. 

1. Fetch - Criteria+Protocol
2. Parse - Parser
3. Store - Updater

The typical flow is a FileListCriteria is built given a date range that 
returns a list of (url, cache_file_location) tuples. The UrlProtocol will 
perform a HTTP GET for each url and write the contents to a file at the
specified cache_file_location. Next the parser is called for each cached file
to open and parse returning a list of dicts for the updater to store. A default
updater is provided called StoreItems that simply stores the items to a list. A
user is expected to provide their own updater.
 
It is sometimes useful to bypass a step but still use the scaffolding. For 
example if you were using OAuth with a WebAPI that returns json and you just
wanted to pass the json respose directly to an Updater to store without using 
a Protocol or Parser. Simply build your feed using the SkipProtocol and 
don't specify a Parser.

.. code-block:: python


    myfeed = build_feed('myfeed', SkipUrlProtocol(MyFetchCriteria()), updater=MyUpdater())
                        

.. toctree::
   :maxdepth: 2



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

