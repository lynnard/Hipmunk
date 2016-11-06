{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TypeSynonymInstances #-}
{-# LANGUAGE TemplateHaskell #-}
-----------------------------------------------------------------------------
-- |
-- Module      :  Physics/Hipmunk/Internal.hsc
-- Copyright   :  (c) 2008-2010 Felipe A. Lessa
-- License     :  MIT (see LICENSE)
--
-- Maintainer  :  felipe.lessa@gmail.com
-- Stability   :  provisional
-- Portability :  portable (needs FFI)
--
-----------------------------------------------------------------------------

module Physics.Hipmunk.Internal
    (VectorPtr,

     BodyPtr,
     Body(..),
     unB,

     Geometry(Circle, LineSegment, Polygon),
     Geometry',
     circleRadius,
     lineStart,
     lineEnd,
     lineThickness,
     polyVertices,
     ofSameType,
     ShapeAttributes(ShapeAttributes),
     ShapeAttributes',
     shapeBody,
     shapeGeometry,
     shapeOffset,
     shapeMass,
     shapeCategoryMask,
     shapeCollisionMask,

     ShapePtr,
     Shape(..),
     unS,


     ConstraintPtr,
     Constraint(..),
     unC,
     Unknown(..),
     ConstraintInit,
     ConstraintType(..),

     SpacePtr,
     Space(..),
     Callbacks(..),
     HandlerFunPtrs,
     unSP,
     freeHandlerFunPtrs,

     unP,

     Entity(..),

     Retrievable(..),
     retrieveShape,
     retrieveBody,
     retrieveConstraint,

     ArbiterPtr,

     Contact(..),
     ContactPtr
    )
    where

import qualified Data.Map as M
import Control.Monad (when)
import Data.IORef
import Data.Map (Map)
import Foreign
#include "wrapper.h"

import Linear.Affine (Point(..))
import Control.Lens

import Physics.Hipmunk.Common


type VectorPtr = Ptr Vector'



-- | A rigid body representing the physical properties of an
--   object, but without a shape. It may help to think of it as a
--   particle that is able to rotate.
newtype Body = B (ForeignPtr Body)
type BodyPtr = Ptr Body

unB :: Body -> ForeignPtr Body
unB (B b) = b

instance Eq Body where
    B b1 == B b2 = b1 == b2

instance Ord Body where
    B b1 `compare` B b2 = b1 `compare` b2



-- | A collision shape is attached to a 'Body' to define its
--   shape. Multiple shapes may be attached, including
--   overlapping ones (shapes of a body don't generate collisions
--   with each other).
--
--   Note that to have any effect, a 'Shape' must also be
--   added to a 'Space', even if the body was already added.

-- | There are three types of shapes that can be attached
--   to bodies:
data Geometry a =
    -- | A circle is the fastest collision type. It also
    --   rolls smoothly.
    Circle {_circleRadius :: !a}

    -- | A line segment is meant to be used as a static
    --   shape. (It can be used with moving bodies, however
    --   two line segments never generate collisions between
    --   each other.)
  | LineSegment {_lineStart     :: !(Position a),
                 _lineEnd       :: !(Position a),
                 _lineThickness :: !a}

    -- | Polygons are the slowest of all shapes but
    --   the most flexible. The list of vertices must form
    --   a convex hull with clockwise winding.
    --   Note that if you want a non-convex polygon you may
    --   add several convex polygons to the body.
  | Polygon {_polyVertices :: ![Position a]}
    deriving (Eq, Ord, Show)

makeLenses ''Geometry

type Geometry' = Geometry CpFloat

ofSameType :: Geometry a -> Geometry a -> Bool
ofSameType (Circle _) (Circle _) = True
ofSameType (LineSegment _ _ _) (LineSegment _ _ _) = True
ofSameType (Polygon _) (Polygon _) = True
ofSameType _ _ = False

instance Functor Geometry where
    fmap f (Circle a) = Circle (f a)
    fmap f (LineSegment start end thick) = LineSegment (f <$> start) (f <$> end) (f thick)
    fmap f (Polygon verts) = Polygon (fmap f <$> verts)

-- TODO: parameterize over double or float?
data ShapeAttributes a = ShapeAttributes
    { _shapeBody :: !Body
    , _shapeGeometry :: !(Geometry a)
    , _shapeOffset :: !(Position a)
    , _shapeMass :: !a
    , _shapeCategoryMask :: !Word64
    , _shapeCollisionMask :: !Word64
    } deriving (Eq, Ord)

makeLenses ''ShapeAttributes

type ShapeAttributes' = ShapeAttributes CpFloat

data Shape = S
    { foreignShape :: !(ForeignPtr Shape)
    , shapeDefRef :: !(IORef ShapeAttributes')
    }

makeLenses ''Shape

type ShapePtr = Ptr Shape

-- Note also that we have to maintain a reference to the
-- 'Body' to avoid garbage collection in the case that
-- the user doesn't add the body to a space and don't keep
-- a reference (common when adding bodies with infinite mass).
--
-- However, the body doesn't need to keep references to
-- the attached shapes because cpBody do not reference them,
-- so it wouldn't notice at all if they disappeared =).
-- A space would notice, but then the space will keep its
-- own reference the the shape.

unS :: Shape -> ForeignPtr Shape
unS (S s _) = s

instance Eq Shape where
    S s1 _ == S s2 _ = s1 == s2

instance Ord Shape where
    S s1 _ `compare` S s2 _ = s1 `compare` s2



-- | Represents a constraint between two bodies. Don't forget to
--   add the bodies and the constraint itself to the space.
--   The phantom type indicates the type of the constraint.
data Constraint a = C !(ForeignPtr (Constraint ())) !Body !Body
type ConstraintPtr = Ptr (Constraint ())

unC :: Constraint a -> ForeignPtr (Constraint ())
unC (C j _ _) = j

instance Eq (Constraint a) where
    C j1 _ _ == C j2 _ _ = j1 == j2

instance Ord (Constraint a) where
    C j1 _ _ `compare` C j2 _ _ = j1 `compare` j2

-- | An unknown constraint \"type\".  Note that this isn't a
--   'ConstraintType' because you can't create a constraint of
--   @Unknown@ type.
data Unknown = Unknown

-- | Type of generic constraint initializar.
type ConstraintInit = ConstraintPtr -> BodyPtr -> BodyPtr -> IO ()

-- | Internal.  Class implemented by all constraint types.
class ConstraintType a where
  size  :: a -> Int
  init_ :: a -> ConstraintInit
  redef :: ConstraintPtr -> Body -> Body -> a -> IO ()


unP :: Position a -> Vector a
unP (P a) = a
{-# INLINE unP #-}


-- | A space is where the simulation really occurs. You add
--   bodies, shapes and constraints to a space and then @step@ it
--   to update it as whole.
data Space = SP !(ForeignPtr Space)
                !(IORef Entities)   -- Active and static entities
                !(IORef Callbacks)  -- Added callbacks
type SpacePtr  = Ptr Space
data Retrievable = ReS Shape | ReB Body | ReC (Constraint Unknown)
type Entities  = Map (Ptr ()) Retrievable
data Callbacks = CBs {cbsDefault  :: HandlerFunPtrs
                     ,cbsHandlers :: Map (CollisionType_, CollisionType_) HandlerFunPtrs
                     ,cbsPostStep :: [FunPtr ()]}
type HandlerFunPtrs = (FunPtr (), FunPtr (), FunPtr (), FunPtr ())
type CollisionType_ = #{type cpCollisionType}
-- Duplicated to avoid bringing the documentation from Shape module.

unSP :: Space -> ForeignPtr Space
unSP (SP sp _ _) = sp

instance Eq Space where
    SP s1 _ _ == SP s2 _ _ = s1 == s2

instance Ord Space where
    SP s1 _ _ `compare` SP s2 _ _ = s1 `compare` s2

-- | Internal. Retrieve a 'Shape' from a 'ShapePtr' and a 'Space'.
retrieveShape :: Space -> ShapePtr -> IO (Maybe Shape)
retrieveShape (SP _ entities _) ptr = do
  ent <- readIORef entities
  case M.lookup (castPtr ptr) ent of
    Just (ReS shape) -> return $ Just shape
    _ -> return Nothing

-- | Internal. Retrieve a 'Body' from a 'BodyPtr' and a 'Space'.
retrieveBody :: Space -> BodyPtr -> IO (Maybe Body)
retrieveBody (SP _ entities _) ptr = do
  ent <- readIORef entities
  case M.lookup (castPtr ptr) ent of
    Just (ReB body) -> return $ Just body
    _ -> return Nothing

-- | Internal. Retrieve a 'Constraint' from a 'ConstraintPtr' and a 'Space'.
retrieveConstraint :: Space -> ConstraintPtr -> IO (Maybe (Constraint Unknown))
retrieveConstraint (SP _ entities _) ptr = do
  ent <- readIORef entities
  case M.lookup (castPtr ptr) ent of
    Just (ReC constraint) -> return $ Just constraint
    _ -> return Nothing


-- | Internal.  Free all function pointers of this handler.
freeHandlerFunPtrs :: HandlerFunPtrs -> IO ()
freeHandlerFunPtrs (p1,p2,p3,p4) = f p1 >> f p2 >> f p3 >> f p4
    where f p = when (p /= nullFunPtr) (freeHaskellFunPtr p)

-- | Type class implemented by entities that can be
--   added to a space.
class Entity a where
    -- | Add an entity to a 'Space'. Don't add the same
    --   entity twice to a space.
    spaceAdd :: Space -> a -> IO ()
    -- | Remove an entity from a 'Space'. Don't remove
    --   an entity that wasn't added.
    spaceRemove :: Space -> a -> IO ()
    -- | Internal function.  Retrive the pointer of this entity.
    entityPtr :: a -> ForeignPtr ()
    -- | Whether the entity is currently in space
    inSpace :: a -> IO Bool


-- | Arbiters are used within callbacks.  We don't expose them to
-- the user.
data Arbiter
type ArbiterPtr = Ptr Arbiter



-- 'Contact's are an exception to the pattern we've been following
-- as we're going to use StorableArray with them, so we need
-- them to be Storable (like Vector).

-- | A 'Contact' contains information about a collision.
--   It is passed to 'Physics.Hipmunk.Space.Full'.
--
--   The fields 'ctJnAcc' and 'ctJtAcc' do not have any meaningfull
--   value until 'Physics.Hipmunk.Space.step' has returned
--   (i.e. during a call to a callback this information
--   contains garbage), and by extension you can only know
--   the impulse sum after @step@ returns as well.
--
--   /IMPORTANT:/ You may maintain a reference to an array of
--   @Contact@s that was passed to a callback to do any other
--   processing later. However, /a new call to/ @step@ /will/
--   /invalidate any of those arrays!/ Be careful.
data Contact = Contact {
      ctPos    :: Position',
      -- ^ Position of the collision in world's coordinates.

      ctNormal :: Vector',
      -- ^ Normal of the collision.

      ctDist   :: CpFloat,
      -- ^ Penetration distance of the collision.

      ctJnAcc  :: CpFloat,
      -- ^ Normal component of final impulse applied.
      --   (Valid only after @step@ finishes.)

      ctJtAcc  :: CpFloat
      -- ^ Tangential component of final impulse applied.
      --   (Valid only after @step@ finishes.)
    }
               deriving (Eq, Ord, Show)

type ContactPtr = Ptr Contact

instance Storable Contact where
    sizeOf _    = #{size cpContact}
    alignment _ = alignment (undefined :: Vector')
    peek ptr    = do
      p     <- #{peek cpContact, p} ptr
      n     <- #{peek cpContact, n} ptr
      dist  <- #{peek cpContact, dist} ptr
      jnAcc <- #{peek cpContact, jnAcc} ptr
      jtAcc <- #{peek cpContact, jtAcc} ptr
      return $ Contact {ctPos    = p
                       ,ctNormal = n
                       ,ctDist   = dist
                       ,ctJnAcc  = jnAcc
                       ,ctJtAcc  = jtAcc}
    poke ptr c = do
      #{poke cpContact, p} ptr (ctPos c)
      #{poke cpContact, n} ptr (ctNormal c)
      #{poke cpContact, dist} ptr (ctDist c)
      #{poke cpContact, jnAcc} ptr (ctJnAcc c)
      #{poke cpContact, jtAcc} ptr (ctJtAcc c)
