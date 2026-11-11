import uuid

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload

from sitesense.db.models.virtual_tour import CaptureStatus, VirtualTour, VirtualTourCapture


class VirtualTourRepository:
    """Database access only; capture policy remains in the service."""

    def __init__(self, session: AsyncSession) -> None:
        self._session = session

    async def create(
        self, *, property_id: uuid.UUID, owner_id: uuid.UUID, expected_capture_count: int
    ) -> VirtualTour:
        tour = VirtualTour(
            id=uuid.uuid4(),
            property_id=property_id,
            owner_id=owner_id,
            expected_capture_count=expected_capture_count,
        )
        self._session.add(tour)
        await self._session.commit()
        reloaded = await self._with_captures(tour.id)
        assert reloaded is not None  # just inserted, on this connection
        return reloaded

    async def get_for_owner(self, tour_id: uuid.UUID, owner_id: uuid.UUID) -> VirtualTour | None:
        tour = await self._with_captures(tour_id)
        return tour if tour is not None and tour.owner_id == owner_id else None

    async def _with_captures(self, tour_id: uuid.UUID) -> VirtualTour | None:
        """Read a tour with its captures already populated.

        `session.refresh()` leaves the relationship expired, and the next
        `tour.captures` is lazy IO — which `AsyncSession` refuses with
        `MissingGreenlet` instead of loading it. Every `TourResponse` counts
        that collection, so it has to come back in the same SELECT.
        """
        result = await self._session.execute(
            select(VirtualTour)
            .options(selectinload(VirtualTour.captures))
            .where(VirtualTour.id == tour_id)
        )
        return result.scalar_one_or_none()

    async def get_capture(
        self, tour_id: uuid.UUID, target_index: int
    ) -> VirtualTourCapture | None:
        result = await self._session.execute(
            select(VirtualTourCapture).where(
                VirtualTourCapture.tour_id == tour_id,
                VirtualTourCapture.target_index == target_index,
            )
        )
        return result.scalar_one_or_none()

    async def add_capture(self, capture: VirtualTourCapture) -> VirtualTourCapture:
        self._session.add(capture)
        await self._session.commit()
        await self._session.refresh(capture)
        return capture

    async def mark_stitch_pending(self, tour: VirtualTour, manifest_key: str) -> VirtualTour:
        tour.status = CaptureStatus.stitch_pending
        tour.stitch_manifest_key = manifest_key
        await self._session.commit()
        reloaded = await self._with_captures(tour.id)
        assert reloaded is not None  # the caller read this row moments ago
        return reloaded
