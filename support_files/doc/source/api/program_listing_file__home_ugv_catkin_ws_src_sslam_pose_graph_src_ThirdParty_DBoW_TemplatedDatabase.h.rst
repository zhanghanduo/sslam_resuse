
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_TemplatedDatabase.h:

Program Listing for File TemplatedDatabase.h
============================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_TemplatedDatabase.h>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/ThirdParty/DBoW/TemplatedDatabase.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #ifndef __D_T_TEMPLATED_DATABASE__
   #define __D_T_TEMPLATED_DATABASE__
   
   #include <vector>
   #include <numeric>
   #include <fstream>
   #include <string>
   #include <list>
   #include <set>
   
   #include "TemplatedVocabulary.h"
   #include "QueryResults.h"
   #include "ScoringObject.h"
   #include "BowVector.h"
   #include "FeatureVector.h"
   
   #include "../DUtils/DUtils.h"
   #include "../../utility/cerealArchiver.h"
   
   namespace DBoW2 {
   
   // For query functions
   static int MIN_COMMON_WORDS = 5;
   
   template<class TDescriptor, class F>
   class TemplatedDatabase
   {
   public:
   
     explicit TemplatedDatabase(bool use_di = true, int di_levels = 0);
   
     template<class T>
     explicit TemplatedDatabase(const T &voc, bool use_di = true, 
       int di_levels = 0);
   
     TemplatedDatabase(const TemplatedDatabase<TDescriptor, F> &db);
   
     TemplatedDatabase(const std::string &filename);
   
     TemplatedDatabase(const char *filename);
   
     virtual ~TemplatedDatabase(void);
   
     TemplatedDatabase<TDescriptor,F>& operator=(
       const TemplatedDatabase<TDescriptor,F> &db);
   
     template<class T>
     inline void setVocabulary(const T &voc);
     
     template<class T>
     void setVocabulary(const T& voc, bool use_di, int di_levels = 0);
     
     inline const TemplatedVocabulary<TDescriptor,F>* getVocabulary() const;
   
     void allocate(int nd = 0, int ni = 0);
   
     EntryId add(const std::vector<TDescriptor> &features,
       BowVector *bowvec = nullptr, FeatureVector *fvec = nullptr);
   
     EntryId add(const BowVector &vec, 
       const FeatureVector &fec = FeatureVector() );
   
     void delete_entry(const EntryId entry_id);
   
     inline void clear();
   
     inline unsigned int size() const;
     
     inline bool usingDirectIndex() const;
     
     inline int getDirectIndexLevels() const;
     
     void query(const std::vector<TDescriptor> &features, QueryResults &ret,
       int max_results = 1, int max_id = -1) const;
     
     void query(const BowVector &vec, QueryResults &ret, 
       int max_results = 1, int max_id = -1) const;
   
     const FeatureVector& retrieveFeatures(EntryId id) const;
   
     void save(const std::string &filename) const;
     
     void load(const std::string &filename);
     
     virtual void save(cv::FileStorage &fs, 
       const std::string &name = "database") const;
     
     virtual void load(const cv::FileStorage &fs, 
       const std::string &name = "database");
   
   protected:
     
     void queryL1(const BowVector &vec, QueryResults &ret, 
       int max_results, int max_id) const;
     
     void queryL2(const BowVector &vec, QueryResults &ret, 
       int max_results, int max_id) const;
     
     void queryChiSquare(const BowVector &vec, QueryResults &ret, 
       int max_results, int max_id) const;
     
     void queryBhattacharyya(const BowVector &vec, QueryResults &ret, 
       int max_results, int max_id) const;
     
     void queryKL(const BowVector &vec, QueryResults &ret, 
       int max_results, int max_id) const;
     
     void queryDotProduct(const BowVector &vec, QueryResults &ret, 
       int max_results, int max_id) const;
   
   protected:
   
     /* Inverted file declaration */
     
     struct IFPair
     {
       EntryId entry_id;
       
       WordValue word_weight;
       
       IFPair(){}
       
       IFPair(EntryId eid, WordValue wv): entry_id(eid), word_weight(wv) {}
       
       inline bool operator==(EntryId eid) const { return entry_id == eid; }
   
         template <class Archive>
         void serialize( Archive & ar )
         {
           ar (CEREAL_NVP(entry_id), CEREAL_NVP(word_weight));
         }
     };
     
     typedef std::list<IFPair> IFRow;
     // IFRows are sorted in ascending entry_id order
     
     typedef std::vector<IFRow> InvertedFile; 
     // InvertedFile[word_id] --> inverted file of that word
     
     /* Direct file declaration */
   
     typedef std::vector<FeatureVector> DirectFile;
     // DirectFile[entry_id] --> [ directentry, ... ]
   
   protected:
   
     TemplatedVocabulary<TDescriptor, F> *m_voc;
     
     bool m_use_di;
     
     int m_dilevels;
     
     InvertedFile m_ifile;
   
     
     DirectFile m_dfile;
     
     std::vector<BowVector> m_dBowfile;
   
     int m_nentries;
   
       friend class cereal::access;
   
       template <class Archive>
       void serialize( Archive & ar )
       {
         ar (CEREAL_NVP(m_use_di), CEREAL_NVP(m_dilevels),
             CEREAL_NVP(m_ifile), CEREAL_NVP(m_dfile),
             CEREAL_NVP(m_nentries), CEREAL_NVP(m_dBowfile));
       }
   
   };
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   TemplatedDatabase<TDescriptor, F>::TemplatedDatabase
     (bool use_di, int di_levels)
     : m_voc(nullptr), m_use_di(use_di), m_dilevels(di_levels), m_nentries(0)
   {
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   template<class T>
   TemplatedDatabase<TDescriptor, F>::TemplatedDatabase
     (const T &voc, bool use_di, int di_levels)
     : m_voc(nullptr), m_use_di(use_di), m_dilevels(di_levels)
   {
     setVocabulary(voc);
     clear();
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   TemplatedDatabase<TDescriptor,F>::TemplatedDatabase
     (const TemplatedDatabase<TDescriptor,F> &db)
     : m_voc(nullptr)
   {
     *this = db;
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   TemplatedDatabase<TDescriptor, F>::TemplatedDatabase
     (const std::string &filename)
     : m_voc(nullptr)
   {
     load(filename);
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   TemplatedDatabase<TDescriptor, F>::TemplatedDatabase
     (const char *filename)
     : m_voc(nullptr)
   {
     load(filename);
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   TemplatedDatabase<TDescriptor, F>::~TemplatedDatabase()
   {
     delete m_voc;
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   TemplatedDatabase<TDescriptor,F>& TemplatedDatabase<TDescriptor,F>::operator=
     (const TemplatedDatabase<TDescriptor,F> &db)
   {
     if(this != &db)
     {
       m_dfile = db.m_dfile;
       m_dBowfile = db.m_dBowfile;
       m_dilevels = db.m_dilevels;
       m_ifile = db.m_ifile;
       m_nentries = db.m_nentries;
       m_use_di = db.m_use_di;
       setVocabulary(*db.m_voc);
     }
     return *this;
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   EntryId TemplatedDatabase<TDescriptor, F>::add(
     const std::vector<TDescriptor> &features,
     BowVector *bowvec, FeatureVector *fvec)
   {
     BowVector aux;
     BowVector& v = (bowvec ? *bowvec : aux);
     
     if(m_use_di && fvec != nullptr)
     {
       m_voc->transform(features, v, *fvec, m_dilevels); // with features
       return add(v, *fvec);
     }
     else if(m_use_di)
     {
       FeatureVector fv;
       m_voc->transform(features, v, fv, m_dilevels); // with features
       return add(v, fv);
     }
     else if(fvec != nullptr)
     {
       m_voc->transform(features, v, *fvec, m_dilevels); // with features
       return add(v);
     }
     else
     {
       m_voc->transform(features, v); // with features
       return add(v);
     }
   }
   
   // ---------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   EntryId TemplatedDatabase<TDescriptor, F>::add(const BowVector &v,
     const FeatureVector &fv)
   {
     EntryId entry_id = m_nentries++;
   
     BowVector::const_iterator vit;
     std::vector<unsigned int>::const_iterator iit;
   
     if(m_use_di)
     {
       // update direct file
       if(entry_id == m_dfile.size())
       {
         m_dfile.push_back(fv);
         m_dBowfile.push_back(v);
       }
       else
       {
         m_dfile[entry_id] = fv;
         m_dBowfile[entry_id] = v;
       }
     }
     
     // update inverted file
     for(vit = v.begin(); vit != v.end(); ++vit)
     {
       const WordId& word_id = vit->first;
       const WordValue& word_weight = vit->second;
       
       IFRow& ifrow = m_ifile[word_id];
       ifrow.push_back(IFPair(entry_id, word_weight));
     }
     
     return entry_id;
   }
   
   // ---------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   void TemplatedDatabase<TDescriptor, F>::delete_entry(const EntryId entry_id)
   {
     BowVector v = m_dBowfile[entry_id];
   
     BowVector::const_iterator vit;
   
     for (vit = v.begin(); vit != v.end(); ++vit)
     {
       const WordId& word_id = vit->first;
       IFRow& ifrow = m_ifile[word_id];
       typename IFRow::iterator rit;
       for (rit = ifrow.begin(); rit != ifrow.end(); ++rit)
       {
         if (rit->entry_id == entry_id)
         {
           ifrow.erase(rit);
           break;
         }
       }
     }
     m_dBowfile[entry_id].clear();
     m_dfile[entry_id].clear();
   }
   
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   template<class T>
   inline void TemplatedDatabase<TDescriptor, F>::setVocabulary
     (const T& voc)
   {
     delete m_voc;
     m_voc = new T(voc);
     clear();
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   template<class T>
   inline void TemplatedDatabase<TDescriptor, F>::setVocabulary
     (const T& voc, bool use_di, int di_levels)
   {
     m_use_di = use_di;
     m_dilevels = di_levels;
     delete m_voc;
     m_voc = new T(voc);
     clear();
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   inline const TemplatedVocabulary<TDescriptor,F>* 
   TemplatedDatabase<TDescriptor, F>::getVocabulary() const
   {
     return m_voc;
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   inline void TemplatedDatabase<TDescriptor, F>::clear()
   {
     // resize vectors
     m_ifile.resize(0);
     m_ifile.resize(m_voc->size());
     m_dfile.resize(0);
     m_dBowfile.resize(0);
     m_nentries = 0;
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   void TemplatedDatabase<TDescriptor, F>::allocate(int nd, int ni)
   {
     // m_ifile already contains |words| items
     if(ni > 0)
     {
       typename std::vector<IFRow>::iterator rit;
       for(rit = m_ifile.begin(); rit != m_ifile.end(); ++rit)
       {
         int n = (int)rit->size();
         if(ni > n)
         {
           rit->resize(ni);
           rit->resize(n);
         }
       }
     }
     
     if(m_use_di && (int)m_dfile.size() < nd)
     {
       m_dfile.resize(nd);
       m_dBowfile.resize(nd);
     }
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   inline unsigned int TemplatedDatabase<TDescriptor, F>::size() const
   {
     return m_nentries;
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   inline bool TemplatedDatabase<TDescriptor, F>::usingDirectIndex() const
   {
     return m_use_di;
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   inline int TemplatedDatabase<TDescriptor, F>::getDirectIndexLevels() const
   {
     return m_dilevels;
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   void TemplatedDatabase<TDescriptor, F>::query(
     const std::vector<TDescriptor> &features,
     QueryResults &ret, int max_results, int max_id) const
   {
     BowVector vec;
     m_voc->transform(features, vec);
     query(vec, ret, max_results, max_id);
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   void TemplatedDatabase<TDescriptor, F>::query(
     const BowVector &vec, 
     QueryResults &ret, int max_results, int max_id) const
   {
     ret.resize(0);
     
     switch(m_voc->getScoringType())
     {
       case L1_NORM:
         queryL1(vec, ret, max_results, max_id);
         break;
         
       case L2_NORM:
         queryL2(vec, ret, max_results, max_id);
         break;
         
       case CHI_SQUARE:
         queryChiSquare(vec, ret, max_results, max_id);
         break;
         
       case KL:
         queryKL(vec, ret, max_results, max_id);
         break;
         
       case BHATTACHARYYA:
         queryBhattacharyya(vec, ret, max_results, max_id);
         break;
         
       case DOT_PRODUCT:
         queryDotProduct(vec, ret, max_results, max_id);
         break;
     }
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   void TemplatedDatabase<TDescriptor, F>::queryL1(const BowVector &vec, 
     QueryResults &ret, int max_results, int max_id) const
   {
     BowVector::const_iterator vit;
     typename IFRow::const_iterator rit;
       
     std::map<EntryId, double> pairs;
     std::map<EntryId, double>::iterator pit;
     
     for(vit = vec.begin(); vit != vec.end(); ++vit)
     {
       const WordId word_id = vit->first;
       const WordValue& qvalue = vit->second;
           
       const IFRow& row = m_ifile[word_id];
       
       // IFRows are sorted in ascending entry_id order
       
       for(rit = row.begin(); rit != row.end(); ++rit)
       {
         const EntryId entry_id = rit->entry_id;
         const WordValue& dvalue = rit->word_weight;
         
         if((int)entry_id < max_id || max_id == -1 || (int)entry_id == m_nentries - 1)
         {
           double value = fabs(qvalue - dvalue) - fabs(qvalue) - fabs(dvalue);
           
           pit = pairs.lower_bound(entry_id);
           if(pit != pairs.end() && !(pairs.key_comp()(entry_id, pit->first)))
           {
             pit->second += value;
           }
           else
           {
             pairs.insert(pit, 
               std::map<EntryId, double>::value_type(entry_id, value));
           }
         }
         
       } // for each inverted row
     } // for each query word
       
     // move to vector
     ret.reserve(pairs.size());
     for(pit = pairs.begin(); pit != pairs.end(); ++pit)
     {
       ret.push_back(Result(pit->first, pit->second));
     }
       
     // resulting "scores" are now in [-2 best .. 0 worst] 
     
     // sort vector in ascending order of score
     std::sort(ret.begin(), ret.end());
     // (ret is inverted now --the lower the better--)
   
     // cut vector
     if(max_results > 0 && (int)ret.size() > max_results)
       ret.resize(max_results);
     
     // complete and scale score to [0 worst .. 1 best]
     // ||v - w||_{L1} = 2 + Sum(|v_i - w_i| - |v_i| - |w_i|) 
     //        for all i | v_i != 0 and w_i != 0 
     // (Nister, 2006)
     // scaled_||v - w||_{L1} = 1 - 0.5 * ||v - w||_{L1}
     QueryResults::iterator qit;
     for(qit = ret.begin(); qit != ret.end(); qit++) 
       qit->Score = -qit->Score/2.0;
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   void TemplatedDatabase<TDescriptor, F>::queryL2(const BowVector &vec, 
     QueryResults &ret, int max_results, int max_id) const
   {
     BowVector::const_iterator vit;
     typename IFRow::const_iterator rit;
     
     std::map<EntryId, double> pairs;
     std::map<EntryId, double>::iterator pit;
     
     //map<EntryId, int> counters;
     //map<EntryId, int>::iterator cit;
     
     for(vit = vec.begin(); vit != vec.end(); ++vit)
     {
       const WordId word_id = vit->first;
       const WordValue& qvalue = vit->second;
       
       const IFRow& row = m_ifile[word_id];
       
       // IFRows are sorted in ascending entry_id order
       
       for(rit = row.begin(); rit != row.end(); ++rit)
       {
         const EntryId entry_id = rit->entry_id;
         const WordValue& dvalue = rit->word_weight;
         
         if((int)entry_id < max_id || max_id == -1)
         {
           double value = - qvalue * dvalue; // minus sign for sorting trick
           
           pit = pairs.lower_bound(entry_id);
           //cit = counters.lower_bound(entry_id);
           if(pit != pairs.end() && !(pairs.key_comp()(entry_id, pit->first)))
           {
             pit->second += value; 
             //cit->second += 1;
           }
           else
           {
             pairs.insert(pit, 
               std::map<EntryId, double>::value_type(entry_id, value));
             
             //counters.insert(cit, 
             //  map<EntryId, int>::value_type(entry_id, 1));
           }
         }
         
       } // for each inverted row
     } // for each query word
       
     // move to vector
     ret.reserve(pairs.size());
     //cit = counters.begin();
     for(pit = pairs.begin(); pit != pairs.end(); ++pit)//, ++cit)
     {
       ret.push_back(Result(pit->first, pit->second));// / cit->second));
     }
       
     // resulting "scores" are now in [-1 best .. 0 worst] 
     
     // sort vector in ascending order of score
     std::sort(ret.begin(), ret.end());
     // (ret is inverted now --the lower the better--)
   
     // cut vector
     if(max_results > 0 && (int)ret.size() > max_results)
       ret.resize(max_results);
   
     // complete and scale score to [0 worst .. 1 best]
     // ||v - w||_{L2} = sqrt( 2 - 2 * Sum(v_i * w_i) 
       //      for all i | v_i != 0 and w_i != 0 )
       // (Nister, 2006)
       QueryResults::iterator qit;
     for(qit = ret.begin(); qit != ret.end(); qit++) 
     {
       if(qit->Score <= -1.0) // rounding error
         qit->Score = 1.0;
       else
         qit->Score = 1.0 - sqrt(1.0 + qit->Score); // [0..1]
         // the + sign is ok, it is due to - sign in 
         // value = - qvalue * dvalue
     }
     
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   void TemplatedDatabase<TDescriptor, F>::queryChiSquare(const BowVector &vec, 
     QueryResults &ret, int max_results, int max_id) const
   {
     BowVector::const_iterator vit;
     typename IFRow::const_iterator rit;
     
     std::map<EntryId, std::pair<double, int> > pairs;
     std::map<EntryId, std::pair<double, int> >::iterator pit;
     
     std::map<EntryId, std::pair<double, double> > sums; // < sum vi, sum wi >
     std::map<EntryId, std::pair<double, double> >::iterator sit;
     
     // In the current implementation, we suppose vec is not normalized
     
     //map<EntryId, double> expected;
     //map<EntryId, double>::iterator eit;
     
     for(vit = vec.begin(); vit != vec.end(); ++vit)
     {
       const WordId word_id = vit->first;
       const WordValue& qvalue = vit->second;
       
       const IFRow& row = m_ifile[word_id];
       
       // IFRows are sorted in ascending entry_id order
       
       for(rit = row.begin(); rit != row.end(); ++rit)
       {
         const EntryId entry_id = rit->entry_id;
         const WordValue& dvalue = rit->word_weight;
         
         if((int)entry_id < max_id || max_id == -1)
         {
           // (v-w)^2/(v+w) - v - w = -4 vw/(v+w)
           // we move the 4 out
           double value = 0;
           if(qvalue + dvalue != 0.0) // words may have weight zero
             value = - qvalue * dvalue / (qvalue + dvalue);
           
           pit = pairs.lower_bound(entry_id);
           sit = sums.lower_bound(entry_id);
           //eit = expected.lower_bound(entry_id);
           if(pit != pairs.end() && !(pairs.key_comp()(entry_id, pit->first)))
           {
             pit->second.first += value;
             pit->second.second += 1;
             //eit->second += dvalue;
             sit->second.first += qvalue;
             sit->second.second += dvalue;
           }
           else
           {
             pairs.insert(pit, 
               std::map<EntryId, std::pair<double, int> >::value_type(entry_id,
                 std::make_pair(value, 1) ));
             //expected.insert(eit, 
             //  map<EntryId, double>::value_type(entry_id, dvalue));
             
             sums.insert(sit, 
               std::map<EntryId, std::pair<double, double> >::value_type(entry_id,
                 std::make_pair(qvalue, dvalue) ));
           }
         }
         
       } // for each inverted row
     } // for each query word
       
     // move to vector
     ret.reserve(pairs.size());
     sit = sums.begin();
     for(pit = pairs.begin(); pit != pairs.end(); ++pit, ++sit)
     {
       if(pit->second.second >= MIN_COMMON_WORDS)
       {
         ret.push_back(Result(pit->first, pit->second.first));
         ret.back().nWords = pit->second.second;
         ret.back().sumCommonVi = sit->second.first;
         ret.back().sumCommonWi = sit->second.second;
         ret.back().expectedChiScore = 
           2 * sit->second.second / (1 + sit->second.second);
       }
     
       //ret.push_back(Result(pit->first, pit->second));
     }
       
     // resulting "scores" are now in [-2 best .. 0 worst] 
     // we have to add +2 to the scores to obtain the chi square score
     
     // sort vector in ascending order of score
     std::sort(ret.begin(), ret.end());
     // (ret is inverted now --the lower the better--)
   
     // cut vector
     if(max_results > 0 && (int)ret.size() > max_results)
       ret.resize(max_results);
   
     // complete and scale score to [0 worst .. 1 best]
     QueryResults::iterator qit;
     for(qit = ret.begin(); qit != ret.end(); qit++)
     {
       // this takes the 4 into account
       qit->Score = - 2. * qit->Score; // [0..1]
       
       qit->chiScore = qit->Score;
     }
     
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   void TemplatedDatabase<TDescriptor, F>::queryKL(const BowVector &vec, 
     QueryResults &ret, int max_results, int max_id) const
   {
     BowVector::const_iterator vit;
     typename IFRow::const_iterator rit;
     
     std::map<EntryId, double> pairs;
     std::map<EntryId, double>::iterator pit;
     
     for(vit = vec.begin(); vit != vec.end(); ++vit)
     {
       const WordId word_id = vit->first;
       const WordValue& vi = vit->second;
       
       const IFRow& row = m_ifile[word_id];
       
       // IFRows are sorted in ascending entry_id order
       
       for(rit = row.begin(); rit != row.end(); ++rit)
       {    
         const EntryId entry_id = rit->entry_id;
         const WordValue& wi = rit->word_weight;
         
         if((int)entry_id < max_id || max_id == -1)
         {
           double value = 0;
           if(vi != 0 && wi != 0) value = vi * log(vi/wi);
           
           pit = pairs.lower_bound(entry_id);
           if(pit != pairs.end() && !(pairs.key_comp()(entry_id, pit->first)))
           {
             pit->second += value;
           }
           else
           {
             pairs.insert(pit, 
               std::map<EntryId, double>::value_type(entry_id, value));
           }
         }
         
       } // for each inverted row
     } // for each query word
       
     // resulting "scores" are now in [-X worst .. 0 best .. X worst]
     // but we cannot make sure which ones are better without calculating
     // the complete score
   
     // complete scores and move to vector
     ret.reserve(pairs.size());
     for(pit = pairs.begin(); pit != pairs.end(); ++pit)
     {
       EntryId eid = pit->first;
       double value = 0.0;
   
       for(vit = vec.begin(); vit != vec.end(); ++vit)
       {
         const WordValue &vi = vit->second;
         const IFRow& row = m_ifile[vit->first];
   
         if(vi != 0)
         {
           if(row.end() == find(row.begin(), row.end(), eid ))
           {
             value += vi * (log(vi) - GeneralScoring::LOG_EPS);
           }
         }
       }
       
       pit->second += value;
       
       // to vector
       ret.push_back(Result(pit->first, pit->second));
     }
     
     // real scores are now in [0 best .. X worst]
   
     // sort vector in ascending order
     // (scores are inverted now --the lower the better--)
     std::sort(ret.begin(), ret.end());
   
     // cut vector
     if(max_results > 0 && (int)ret.size() > max_results)
       ret.resize(max_results);
   
     // cannot scale scores
       
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   void TemplatedDatabase<TDescriptor, F>::queryBhattacharyya(
     const BowVector &vec, QueryResults &ret, int max_results, int max_id) const
   {
     BowVector::const_iterator vit;
     typename IFRow::const_iterator rit;
     
     //map<EntryId, double> pairs;
     //map<EntryId, double>::iterator pit;
     
     std::map<EntryId, std::pair<double, int> > pairs; // <eid, <score, counter> >
     std::map<EntryId, std::pair<double, int> >::iterator pit;
     
     for(vit = vec.begin(); vit != vec.end(); ++vit)
     {
       const WordId word_id = vit->first;
       const WordValue& qvalue = vit->second;
       
       const IFRow& row = m_ifile[word_id];
       
       // IFRows are sorted in ascending entry_id order
       
       for(rit = row.begin(); rit != row.end(); ++rit)
       {
         const EntryId entry_id = rit->entry_id;
         const WordValue& dvalue = rit->word_weight;
         
         if((int)entry_id < max_id || max_id == -1)
         {
           double value = sqrt(qvalue * dvalue);
           
           pit = pairs.lower_bound(entry_id);
           if(pit != pairs.end() && !(pairs.key_comp()(entry_id, pit->first)))
           {
             pit->second.first += value;
             pit->second.second += 1;
           }
           else
           {
             pairs.insert(pit, 
               std::map<EntryId, std::pair<double, int> >::value_type(entry_id,
                 std::make_pair(value, 1)));
           }
         }
         
       } // for each inverted row
     } // for each query word
       
     // move to vector
     ret.reserve(pairs.size());
     for(pit = pairs.begin(); pit != pairs.end(); ++pit)
     {
       if(pit->second.second >= MIN_COMMON_WORDS)
       {
         ret.push_back(Result(pit->first, pit->second.first));
         ret.back().nWords = pit->second.second;
         ret.back().bhatScore = pit->second.first;
       }
     }
       
     // scores are already in [0..1]
   
     // sort vector in descending order
     std::sort(ret.begin(), ret.end(), Result::gt);
   
     // cut vector
     if(max_results > 0 && (int)ret.size() > max_results)
       ret.resize(max_results);
   
   }
   
   // ---------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   void TemplatedDatabase<TDescriptor, F>::queryDotProduct(
     const BowVector &vec, QueryResults &ret, int max_results, int max_id) const
   {
     BowVector::const_iterator vit;
     typename IFRow::const_iterator rit;
     
     std::map<EntryId, double> pairs;
     std::map<EntryId, double>::iterator pit;
     
     for(vit = vec.begin(); vit != vec.end(); ++vit)
     {
       const WordId word_id = vit->first;
       const WordValue& qvalue = vit->second;
       
       const IFRow& row = m_ifile[word_id];
       
       // IFRows are sorted in ascending entry_id order
       
       for(rit = row.begin(); rit != row.end(); ++rit)
       {
         const EntryId entry_id = rit->entry_id;
         const WordValue& dvalue = rit->word_weight;
         
         if((int)entry_id < max_id || max_id == -1)
         {
           double value; 
           if(this->m_voc->getWeightingType() == BINARY)
             value = 1;
           else
             value = qvalue * dvalue;
           
           pit = pairs.lower_bound(entry_id);
           if(pit != pairs.end() && !(pairs.key_comp()(entry_id, pit->first)))
           {
             pit->second += value;
           }
           else
           {
             pairs.insert(pit, 
               std::map<EntryId, double>::value_type(entry_id, value));
           }
         }
         
       } // for each inverted row
     } // for each query word
       
     // move to vector
     ret.reserve(pairs.size());
     for(pit = pairs.begin(); pit != pairs.end(); ++pit)
     {
       ret.push_back(Result(pit->first, pit->second));
     }
       
     // scores are the greater the better
   
     // sort vector in descending order
     std::sort(ret.begin(), ret.end(), Result::gt);
   
     // cut vector
     if(max_results > 0 && (int)ret.size() > max_results)
       ret.resize(max_results);
   
     // these scores cannot be scaled
   }
   
   // ---------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   const FeatureVector& TemplatedDatabase<TDescriptor, F>::retrieveFeatures
     (EntryId id) const
   {
     assert(id < size());
     return m_dfile[id];
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   void TemplatedDatabase<TDescriptor, F>::save(const std::string &filename) const
   {
     cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
     if(!fs.isOpened()) throw std::string("Could not open file ") + filename;
     
     save(fs);
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   void TemplatedDatabase<TDescriptor, F>::save(cv::FileStorage &fs,
     const std::string &name) const
   {
     // Format YAML:
     // vocabulary { ... see TemplatedVocabulary::save }
     // database 
     // {
     //   nEntries: 
     //   usingDI: 
     //   diLevels: 
     //   invertedIndex
     //   [
     //     [
     //        { 
     //          imageId: 
     //          weight: 
     //        }
     //     ]
     //   ]
     //   directIndex
     //   [
     //      [
     //        {
     //          nodeId:
     //          features: [ ]
     //        }
     //      ]
     //   ]
   
     // invertedIndex[i] is for the i-th word
     // directIndex[i] is for the i-th entry
     // directIndex may be empty if not using direct index
     //
     // imageId's and nodeId's must be stored in ascending order
     // (according to the construction of the indexes)
   
   //  m_voc->save(fs);
    
     fs << name << "{";
     
     fs << "nEntries" << m_nentries;
     fs << "usingDI" << (m_use_di ? 1 : 0);
     fs << "diLevels" << m_dilevels;
     
     fs << "invertedIndex" << "[";
     
     typename InvertedFile::const_iterator iit;
     typename IFRow::const_iterator irit;
     for(iit = m_ifile.begin(); iit != m_ifile.end(); ++iit)
     {
       fs << "["; // word of IF
       for(irit = iit->begin(); irit != iit->end(); ++irit)
       {
         fs << "{:" 
           << "imageId" << (int)irit->entry_id
           << "weight" << irit->word_weight
           << "}";
       }
       fs << "]"; // word of IF
     }
     
     fs << "]"; // invertedIndex
     
     fs << "directIndex" << "[";
     
     typename DirectFile::const_iterator dit;
     typename FeatureVector::const_iterator drit;
     for(dit = m_dfile.begin(); dit != m_dfile.end(); ++dit)
     {
       fs << "["; // entry of DF
       
       for(drit = dit->begin(); drit != dit->end(); ++drit)
       {
         NodeId nid = drit->first;
         const std::vector<unsigned int>& features = drit->second;
         
         // save info of last_nid
         fs << "{";
         fs << "nodeId" << (int)nid;
         // msvc++ 2010 with opencv 2.3.1 does not allow FileStorage::operator<<
         // with vectors of unsigned int
         fs << "features" << "[" 
           << *(const std::vector<int>*)(&features) << "]";
         fs << "}";
       }
       
       fs << "]"; // entry of DF
     }
     
     fs << "]"; // directIndex
     
     fs << "}"; // database
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   void TemplatedDatabase<TDescriptor, F>::load(const std::string &filename)
   {
     cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
     if(!fs.isOpened()) throw std::string("Could not open file ") + filename;
     
     load(fs);
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   void TemplatedDatabase<TDescriptor, F>::load(const cv::FileStorage &fs,
     const std::string &name)
   { 
     // load voc first
     // subclasses must instantiate m_voc before calling this ::load
     if(!m_voc) m_voc = new TemplatedVocabulary<TDescriptor, F>;
   
     m_voc->load(fs);
   
     // load database now
     clear(); // resizes inverted file 
       
     cv::FileNode fdb = fs[name];
     
     m_nentries = (int)fdb["nEntries"]; 
     m_use_di = (int)fdb["usingDI"] != 0;
     m_dilevels = (int)fdb["diLevels"];
     
     cv::FileNode fn = fdb["invertedIndex"];
     for(WordId wid = 0; wid < fn.size(); ++wid)
     {
       cv::FileNode fw = fn[wid];
       
       for(unsigned int i = 0; i < fw.size(); ++i)
       {
         EntryId eid = (int)fw[i]["imageId"];
         WordValue v = fw[i]["weight"];
         
         m_ifile[wid].push_back(IFPair(eid, v));
       }
     }
     
     if(m_use_di)
     {
       fn = fdb["directIndex"];
       
       m_dfile.resize(fn.size());
       m_dBowfile.resize(fn.size());
       assert(m_nentries == (int)fn.size());
       
       FeatureVector::iterator dit;
       for(EntryId eid = 0; eid < fn.size(); ++eid)
       {
         cv::FileNode fe = fn[eid];
         
         m_dfile[eid].clear();
         m_dBowfile[eid].clear();
         for(unsigned int i = 0; i < fe.size(); ++i)
         {
           NodeId nid = (int)fe[i]["nodeId"];
           
           dit = m_dfile[eid].insert(m_dfile[eid].end(), 
             make_pair(nid, std::vector<unsigned int>() ));
           
           // this failed to compile with some opencv versions (2.3.1)
           //fe[i]["features"] >> dit->second;
           
           // this was ok until OpenCV 2.4.1
           //std::vector<int> aux;
           //fe[i]["features"] >> aux; // OpenCV < 2.4.1
           //dit->second.resize(aux.size());
           //std::copy(aux.begin(), aux.end(), dit->second.begin());
           
           cv::FileNode ff = fe[i]["features"][0];
           dit->second.reserve(ff.size());
                   
           cv::FileNodeIterator ffit;
           for(ffit = ff.begin(); ffit != ff.end(); ++ffit)
           {
             dit->second.push_back((int)*ffit); 
           }
         }
       } // for each entry
     } // if use_id
     
   }
   
   // --------------------------------------------------------------------------
   
   template<class TDescriptor, class F>
   std::ostream& operator<<(std::ostream &os, 
     const TemplatedDatabase<TDescriptor,F> &db)
   {
     os << "Database: Entries = " << db.size() << ", "
       "Using direct index = " << (db.usingDirectIndex() ? "yes" : "no");
     
     if(db.usingDirectIndex())
       os << ", Direct index levels = " << db.getDirectIndexLevels();
     
     os << ". " << *db.getVocabulary();
     return os;
   }
   
   // --------------------------------------------------------------------------
   
   } // namespace DBoW2
   
   #endif
